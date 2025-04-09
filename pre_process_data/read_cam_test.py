import h5py
import numpy as np
import cv2

def extract_valid_jpeg_bytes(row):
    """
    从一行字节数组中提取有效的 JPEG 数据。
    遍历字节寻找 JPEG 文件的结尾标志 [255, 217]，
    并返回有效的部分（包含该标志）。
    """
    for i in range(len(row) - 1):
        if row[i] == 255 and row[i+1] == 217:
            return row[:i+2]
    return row

# 读取 HDF5 文件
with h5py.File('data.h5', 'r') as f:
    cam_frames_padded = f['cam_frames'][:]  # 得到二维数组：每行存储一帧 JPEG 编码后的字节

# 对每一帧数据进行处理和解码
decoded_frames = []
for i, row in enumerate(cam_frames_padded):
    valid_bytes = extract_valid_jpeg_bytes(row)
    img = cv2.imdecode(valid_bytes, cv2.IMREAD_COLOR)
    if img is None:
        print(f"Frame {i} 解码失败！")
    else:
        decoded_frames.append(img)

print("解码成功的帧数：", len(decoded_frames))

# 逐张显示图像，按任意键继续显示下一张
for idx, frame in enumerate(decoded_frames):
    cv2.imshow("Image", frame)
    print(f"显示第 {idx+1} 帧，按任意键继续，按 'q' 键退出...")
    key = cv2.waitKey(0)  # 等待按键
    # 如果按下 'q' 键则退出展示
    if key & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
