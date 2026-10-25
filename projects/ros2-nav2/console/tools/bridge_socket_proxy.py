#!/usr/bin/env python3
"""Expose the stdio ROS bridge through a local Unix socket."""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any


class BridgeSocketProxy:
    def __init__(
        self,
        bridge_script: Path,
        socket_path: Path,
        config_json: str,
    ) -> None:
        self.bridge_script = bridge_script
        self.socket_path = socket_path
        self.config_json = config_json
        self.stop_event = threading.Event()
        self.closed = threading.Event()
        self.clients: set[socket.socket] = set()
        self.client_lock = threading.RLock()
        self.process_lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.process: subprocess.Popen[str] | None = None
        self.bridge_lines: queue.Queue[str] = queue.Queue()
        self.last_map_event: dict[str, Any] | None = None
        self.listener = self._create_listener()

    def _create_listener(self) -> socket.socket:
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.bind(str(self.socket_path))
        os.chmod(self.socket_path, 0o660)
        listener.listen(4)
        listener.settimeout(0.5)
        return listener

    def serve(self) -> None:
        threading.Thread(
            target=self._accept_loop,
            name="socket-accept",
            daemon=True,
        ).start()
        try:
            self._bridge_loop()
        finally:
            self.close()

    def _accept_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                connection, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self.client_lock:
                self.clients.add(connection)
            if self.last_map_event is not None:
                self._send_json(connection, self.last_map_event)
            threading.Thread(
                target=self._client_loop,
                args=(connection,),
                name="bridge-client",
                daemon=True,
            ).start()

    def _bridge_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                self._run_bridge_once()
            except Exception as exc:  # noqa: BLE001
                print(f"[bridge-proxy] bridge error: {exc}", flush=True)
            if not self.stop_event.is_set():
                self.stop_event.wait(1.0)

    def _run_bridge_once(self) -> None:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        process = subprocess.Popen(
            [sys.executable, str(self.bridge_script), self.config_json],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env=env,
        )
        with self.process_lock:
            self.process = process
        print(f"[bridge-proxy] bridge started pid={process.pid}", flush=True)
        assert process.stdout is not None
        assert process.stderr is not None
        threading.Thread(
            target=self._read_stdout,
            args=(process,),
            name="bridge-stdout",
            daemon=True,
        ).start()
        threading.Thread(
            target=self._read_stderr,
            args=(process,),
            name="bridge-stderr",
            daemon=True,
        ).start()
        try:
            while process.poll() is None or not self.bridge_lines.empty():
                try:
                    line = self.bridge_lines.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(event, dict) and event.get("type") == "map":
                    self.last_map_event = event
                self._broadcast_line(line)
            code = process.wait()
        finally:
            with self.process_lock:
                if self.process is process:
                    self.process = None
        if not self.stop_event.is_set():
            print(f"[bridge-proxy] bridge exited code={code}", flush=True)

    def _read_stdout(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            line = line.rstrip("\r\n")
            if line:
                self.bridge_lines.put(line)

    def _read_stderr(self, process: subprocess.Popen[str]) -> None:
        if process.stderr is None:
            return
        for line in process.stderr:
            line = line.rstrip()
            if line:
                print(f"[bridge-proxy] {line}", flush=True)

    def _client_loop(self, connection: socket.socket) -> None:
        try:
            stream = connection.makefile("r", encoding="utf-8", errors="replace")
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    command = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(command, dict):
                    self.submit(command)
        except OSError:
            pass
        finally:
            with self.client_lock:
                self.clients.discard(connection)
            try:
                connection.close()
            except OSError:
                pass

    def submit(self, command: dict[str, Any]) -> None:
        payload = json.dumps(command, separators=(",", ":")) + "\n"
        with self.write_lock:
            with self.process_lock:
                process = self.process
            if (
                process is None
                or process.poll() is not None
                or process.stdin is None
            ):
                raise RuntimeError("ROS bridge process is not running")
            try:
                process.stdin.write(payload)
                process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as exc:
                raise RuntimeError("ROS bridge process is not writable") from exc

    def _broadcast_line(self, line: str) -> None:
        data = line.encode("utf-8") + b"\n"
        with self.client_lock:
            clients = list(self.clients)
        for connection in clients:
            try:
                connection.sendall(data)
            except OSError:
                with self.client_lock:
                    self.clients.discard(connection)
                try:
                    connection.close()
                except OSError:
                    pass

    def _send_json(
        self,
        connection: socket.socket,
        payload: dict[str, Any],
    ) -> None:
        try:
            connection.sendall(
                (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
            )
        except OSError:
            with self.client_lock:
                self.clients.discard(connection)

    def close(self) -> None:
        if self.closed.is_set():
            return
        self.closed.set()
        self.stop_event.set()
        try:
            self.submit({"action": "shutdown"})
        except RuntimeError:
            pass
        with self.process_lock:
            process = self.process
        if process is not None:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
        try:
            self.listener.close()
        except OSError:
            pass
        with self.client_lock:
            clients = list(self.clients)
            self.clients.clear()
        for connection in clients:
            try:
                connection.close()
            except OSError:
                pass
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("config_json")
    args = parser.parse_args()
    proxy = BridgeSocketProxy(args.bridge, args.socket, args.config_json)

    def request_stop(_signum: int, _frame: Any) -> None:
        proxy.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        proxy.serve()
    except KeyboardInterrupt:
        pass
    finally:
        proxy.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
