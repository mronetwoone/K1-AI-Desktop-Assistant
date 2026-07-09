# 模型文件说明

K1 端程序默认使用 `yolov8n-pose-320.onnx`。

部署时把模型放到 K1 板端，例如：

```bash
/home/lu/Horse/Q_S/model/yolov8n-pose-320.onnx
```

也可以在 `src/k1/main.cpp` 中修改 `model_path` 为自己的实际路径。
