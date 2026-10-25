# COCO FP32/INT8 基线审计

审计日期：2026-10-25

## 结论

当前 FP32/INT8 数值不能直接作为 Ultralytics 官方 COCO 基线，也不能用来比较
“官方 YOLOv8n mAP”。

原因是评估脚本在调用 `pycocotools.COCOeval` 之前，先以默认
`--score-thres 0.25` 过滤低置信候选，再执行 NMS 并写出 COCO 预测 JSON。
`COCOeval` 无法恢复已经被删除的低分框，因此：

- `AR@100=0.331` 不是标准 COCO AR；
- 当前 `AP50:95` 也不是未经截断的标准 COCO AP；
- 两个模型都经过同一阈值，因此这组数据可作为“项目内固定工作点”的配对比较，
  但不能与 Ultralytics 官方低阈值评估表格直接比较。

## 证据

- `run_fp32_coco.py`、`run_fp32_coco_stretch.py`、
  `run_bpu_coco.py`、`run_bpu_coco_stretch.py` 和
  `run_hb_onnx_coco_{fp32,quantized}.py` 的默认值均为 `score_thres=0.25`。
- `fp32_stretch_full_meta.json` 与 `newptq_stretch_full_meta.json`
  均记录 `score_threshold=0.25`、`nms_threshold=0.7`。
- `score_coco.py` 读取的是已经过滤后的预测 JSON；它只执行标准
  `COCOeval.evaluate()`、`accumulate()` 和 `summarize()`，不会补回低分框。

## 仍可支持的表述

简历当前采用 2026-10-23 封版配对：

- FP32：AP50:95 `0.2780403`；
- INT8：AP50:95 `0.2540833`；
- 配对差值：`2.3957 pp`，保留率 `91.38%`。

另一轮 2026-09-18 同源实验在相同截断下得到 FP32/INT8
`0.2780816/0.2535251`、差 `2.4557 pp`、保留率 `91.17%`。两轮都只说明固定工作点下
INT8 与 FP32 的配对差异，不能说明模型达到官方 COCO 水平。`AR@100=0.331`
来自 2026-09-17 的 stretch 全量运行，同样受低分框预过滤影响。

## 修正计划

需要分别重跑 FP32 和 INT8，而不是从现有预测 JSON 修补：

1. 在后处理前使用接近官方评测的低阈值，例如 `score>=0.001`；
2. 保持相同的预处理、NMS 和类别映射；
3. 每图最多保留 COCOeval 使用的 Top-100 预测；
4. 使用相同 `pycocotools` 版本重新执行全量 5,000 图评估；
5. 保存新的预测 JSON、元数据、日志和 AP/AR 汇总后，再更新简历和 README。

在完成重跑前，不应宣称“FP32 基线已复核”或“AP 达到官方水平”。
