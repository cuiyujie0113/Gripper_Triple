import h5py
import numpy as np
import cv2
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
import hydra
from hydra.core.global_hydra import GlobalHydra
import os
import time

def extract_valid_jpeg_bytes(row):
    """
    从一行字节数组中提取有效的 JPEG 数据
    遍历字节寻找 JPEG 文件的结尾标志 [255, 217]，返回有效部分（包含该标志）
    """
    for i in range(len(row) - 1):
        if row[i] == 255 and row[i+1] == 217:
            return row[:i+2]
    return row

# --- 1. 读取 HDF5 文件中的图像数据 ---
with h5py.File('data.h5', 'r') as f:
    cam_frames_padded = f['cam_frames'][:]  # 每行存放一帧 JPEG 字节数据（经过零填充）

decoded_frames = []
for i, row in enumerate(cam_frames_padded):
    valid_bytes = extract_valid_jpeg_bytes(row)
    img = cv2.imdecode(valid_bytes, cv2.IMREAD_COLOR)
    if img is None:
        print(f"Frame {i} 解码失败。")
    else:
        decoded_frames.append(img)

print(f"成功解码 {len(decoded_frames)} 帧。")

# --- 2. 加载 SAM2 预训练模型 ---
# --- 0. 定义路径 ---
config_dir = "sam2_model"  # YAML配置文件所在目录
checkpoint_path = "/media/cyj/DATA/大四下/UMI_Gripper3/Gripper_Triple/pre_process_data/sam2_model/sam2_hiera_tiny.pt"  # 模型权重路径
 
# --- 1. 初始化 Hydra 并添加自定义配置路径 ---
GlobalHydra.instance().clear()  # 清除之前的 Hydra 实例
hydra.initialize(config_path=config_dir)  # 指定 Hydra 的配置搜索路径

# --- 2. 加载 SAM2 预训练模型 ---
try:
    # 加载配置文件（Hydra 会在 config_dir 中查找 sam2_hiera_l.yaml）
    model_cfg = "sam2_hiera_t.yaml"  # 直接使用文件名
    
    # 确保 checkpoint 文件存在
    assert os.path.exists(checkpoint_path), f"Checkpoint {checkpoint_path} 不存在!"
    
    # 构建 SAM2 模型
    sam2_model = build_sam2(model_cfg, checkpoint_path)

        
    # 创建预测器
    predictor = SAM2ImagePredictor(sam2_model)
    
    print("模型加载成功!")
except Exception as e:
    print(f"加载失败: {str(e)}")
finally:
    # 清理 Hydra 配置（可选）
    GlobalHydra.instance().clear()

# --- 3. 逐张分割并展示图像 ---
for idx, frame in enumerate(decoded_frames):
    start_time = time.time()

    # 设置当前图像到 predictor 中
    predictor.set_image(frame)
    height, width, _ = frame.shape
    print(f"\n=== 第 {idx+1} 帧 ===")
    print(f"原始图像形状: {frame.shape} (H, W, C)")  # <-- 检查原图尺寸

    # 使用整个图像的边界框作为提示，格式为 numpy 数组，要求 shape 为 (N, 4)
    box = np.array([[0, 0, width, height]], dtype=np.float32)

    # 模型预测，使用 GPU 加速，如果没有 GPU 可修改为 cpu
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
        # 注意：predictor.predict 接口的参数可能根据 API 不同而略有差异，这里使用 box 输入作为提示
        masks, scores, logits = predictor.predict(box=box)

    # 新增检查点：输出掩膜信息 <--
    print(f"生成的掩膜数量: {len(masks)}")
    if len(masks) > 0:
        print(f"单个掩膜形状: {masks[0].shape} (H, W)")
    
    # --- 叠加分割 mask ---
    overlay = frame.copy()
    for i, m in enumerate(masks):
        # 新增检查点：每个掩膜的形状 <--
        print(f"正在处理第 {i+1} 个掩膜 | 形状: {m.shape}")
        
        color = np.random.randint(0, 256, size=3).tolist()
        mask_bool = m.astype(bool)

        overlay[mask_bool] = (overlay[mask_bool] * 0.5 + np.array(color) * 0.5).astype(np.uint8)
        
        # 检查形状兼容性（调试用）
        print(f"掩膜布尔形状: {mask_bool.shape} | 叠加层形状: {overlay.shape}")
        assert mask_bool.shape == (height, width), f"掩膜尺寸不匹配!"

    # 显示文字和图片（后续代码不变）
    cv2.putText(overlay, f"Frame: {idx+1}, Masks: {len(masks)}", (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Segmented Image", overlay)
    end_time = time.time()
    print(f"处理第 {idx+1} 帧耗时: {end_time - start_time:.2f} 秒")
    print(f"显示第 {idx+1} 帧，按任意键继续，按 'q' 退出...")
    key = cv2.waitKey(0)
    if key & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
