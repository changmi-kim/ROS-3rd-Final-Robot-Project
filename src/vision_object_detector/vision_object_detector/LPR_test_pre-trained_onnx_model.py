import onnxruntime
from PIL import Image
import numpy as np
import cv2

def load_and_infer_onnx_model(model_path, input_image_path):
    session = onnxruntime.InferenceSession(model_path)

    image = Image.open(input_image_path)
    target_size = (640, 640)
    image_resized = image.resize(target_size, Image.ANTIALIAS)
    image_array = np.array(image_resized)
    input_data = image_array.transpose(2, 0, 1).astype(np.float32)
    input_data = np.expand_dims(input_data, axis=0)

    output = session.run(None, {'images': input_data})

    return image, output

def recognize_license_plate(onnx_model_path, image_path):
    image, model_output = load_and_infer_onnx_model(onnx_model_path, image_path)
    bounding_box = extract_bounding_box(model_output)
    if bounding_box:
        visualize_result(image, bounding_box)
        crop_and_show(image, bounding_box)

def extract_bounding_box(model_output):
    bounding_box_data = model_output[0]

    x_min, y_min, x_max, y_max, confidence = bounding_box_data[0][:5]
    print(x_min, y_min, x_max, y_max, confidence)

    return (x_min, y_min, x_max, y_max, confidence)

def visualize_result(image, bounding_box):
    image_width, image_height = image.size
    x_min, y_min, x_max, y_max, _ = bounding_box
    x_min *= image_width
    y_min *= image_height
    x_max *= image_width
    y_max *= image_height

    x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])
    print(x_min, y_min, x_max, y_max)

    x_min, y_min = max(0, x_min), max(0, y_min)
    x_max, y_max = min(image_width, x_max), min(image_height, y_max)

    img_cv2 = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

    cv2.rectangle(img_cv2, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    cv2.imshow('License Plate Detection', img_cv2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def crop_and_show(image, bounding_box):
    image_width, image_height = image.size
    x_min, y_min, x_max, y_max, _ = bounding_box
    x_min *= image_width
    y_min *= image_height
    x_max *= image_width
    y_max *= image_height

    x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])

    x_min, y_min = max(0, x_min), max(0, y_min)
    x_max, y_max = min(image_width, x_max), min(image_height, y_max)

    img_np = np.array(image)

    cropped_image = img_np[y_min:y_max, x_min:x_max, :]

    cv2.imshow('Cropped License Plate', cv2.cvtColor(cropped_image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    yolo_model_path = '/home/ckdal/dev_ws/project/final_project_ws/final_project_ws/yolov8n_plate_detect.onnx'
    # yolo_model_2_path = '/home/ckdal/dev_ws/project/final_project_ws/final_project_ws/yolov8m_number_detect.onnx'
    
    image_path = '/home/ckdal/dev_ws/project/final_project_ws/final_project_ws/LPR_test.jpg'
    
    recognize_license_plate(yolo_model_path, image_path)
