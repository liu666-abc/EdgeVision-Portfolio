#!/usr/bin/env node

import { writeFileSync } from "node:fs";

const url = process.argv[2] || "http://127.0.0.1:8091";
const debugPort = Number(process.argv[3] || 9223);
const viewportWidth = Number(process.argv[4] || 0);
const viewportHeight = Number(process.argv[5] || 0);
const screenshotPath = process.argv[6] || "";
const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function findTarget() {
  for (let attempt = 0; attempt < 30; attempt += 1) {
    try {
      const response = await fetch(`http://127.0.0.1:${debugPort}/json`);
      const targets = await response.json();
      const page = targets.find((target) => target.type === "page");
      if (page) return page;
    } catch {
      // Browser is still starting.
    }
    await wait(200);
  }
  throw new Error("CDP page target not found");
}

function openCdp(webSocketUrl) {
  return new Promise((resolve, reject) => {
    const socket = new WebSocket(webSocketUrl);
    const pending = new Map();
    const events = [];
    let sequence = 0;
    socket.addEventListener("open", () => {
      resolve({
        events,
        send(method, params = {}) {
          sequence += 1;
          const id = sequence;
          return new Promise((commandResolve, commandReject) => {
            pending.set(id, { resolve: commandResolve, reject: commandReject });
            socket.send(JSON.stringify({ id, method, params }));
          });
        },
        close() {
          socket.close();
        },
      });
    });
    socket.addEventListener("message", (event) => {
      const payload = JSON.parse(event.data);
      if (payload.id && pending.has(payload.id)) {
        const command = pending.get(payload.id);
        pending.delete(payload.id);
        if (payload.error) command.reject(new Error(payload.error.message));
        else command.resolve(payload.result);
      } else if (payload.method) {
        events.push(payload);
      }
    });
    socket.addEventListener("error", () => reject(new Error("CDP socket error")));
  });
}

async function evaluate(cdp, expression) {
  const result = await cdp.send("Runtime.evaluate", {
    expression,
    awaitPromise: true,
    returnByValue: true,
  });
  if (result.exceptionDetails) {
    throw new Error(result.exceptionDetails.text || "evaluation failed");
  }
  return result.result?.value;
}

const target = await findTarget();
const cdp = await openCdp(target.webSocketDebuggerUrl);
await cdp.send("Runtime.enable");
await cdp.send("Page.enable");
if (viewportWidth > 0 && viewportHeight > 0) {
  await cdp.send("Emulation.setDeviceMetricsOverride", {
    width: viewportWidth,
    height: viewportHeight,
    deviceScaleFactor: 1,
    mobile: false,
  });
}
await cdp.send("Page.navigate", { url });
await wait(4500);

const layout = await evaluate(cdp, `(() => {
  const stage = document.getElementById("mapStage").getBoundingClientRect();
  const panel = document.querySelector(".control-panel").getBoundingClientRect();
  const canvas = document.getElementById("mapCanvas");
  const pixels = canvas.getContext("2d").getImageData(0, 0, canvas.width, canvas.height).data;
  let nonTransparent = 0;
  for (let i = 3; i < pixels.length; i += 4) {
    if (pixels[i] > 0) nonTransparent += 1;
  }
  return {
    innerWidth,
    innerHeight,
    scrollWidth: document.documentElement.scrollWidth,
    scrollHeight: document.documentElement.scrollHeight,
    mapStage: { x: stage.x, y: stage.y, width: stage.width, height: stage.height },
    controlPanel: { x: panel.x, y: panel.y, width: panel.width, height: panel.height },
    canvasPixels: nonTransparent,
    mapCells: document.getElementById("mapCells").textContent,
    scanPoints: document.getElementById("scanPoints").textContent,
    wsText: document.querySelector("#wsPill em").textContent,
    rosText: document.querySelector("#rosPill em").textContent,
    navText: document.querySelector("#navPill em").textContent,
    actionServerText: document.getElementById("actionServerState").textContent,
    sendGoalDisabled: document.getElementById("sendGoalButton").disabled,
  };
})()`);

await evaluate(cdp, `document.getElementById("safetyConfirm").click()`);
const safetyEnabled = await evaluate(
  cdp,
  `!document.getElementById("sendGoalButton").disabled`,
);

if (screenshotPath) {
  const screenshot = await cdp.send("Page.captureScreenshot", {
    format: "png",
    captureBeyondViewport: false,
  });
  writeFileSync(screenshotPath, Buffer.from(screenshot.data, "base64"));
}

const exceptions = cdp.events
  .filter((event) => event.method === "Runtime.exceptionThrown")
  .map((event) => event.params.exceptionDetails.text);
const logErrors = cdp.events
  .filter((event) => event.method === "Log.entryAdded")
  .map((event) => event.params.entry)
  .filter((entry) => entry.level === "error")
  .map((entry) => entry.text);

console.log(JSON.stringify({
  ok: layout.canvasPixels > 1000
    && layout.wsText === "ONLINE"
    && layout.rosText === "ONLINE"
    && (
      layout.actionServerText === "READY"
        ? safetyEnabled
        : layout.sendGoalDisabled
    )
    && exceptions.length === 0
    && logErrors.length === 0,
  layout,
  safetyEnabled,
  exceptions,
  logErrors,
}, null, 2));

cdp.close();

