import numpy as np
import cv2

# --------------------------------------------------
# CONFIG
# --------------------------------------------------

CONF_THRES = 0.25
IOU_THRES = 0.45

# --------------------------------------------------
# YOLOv11 RKNN DECODER
# --------------------------------------------------

def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # x1
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # y1
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # x2
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # y2
    return y


def nms_boxes(boxes, scores, iou_thres):
    idxs = cv2.dnn.NMSBoxes(
        boxes.tolist(),
        scores.tolist(),
        score_threshold=0,
        nms_threshold=iou_thres
    )
    if len(idxs) == 0:
        return []

    return idxs.flatten()


# --------------------------------------------------
# MAIN POSTPROCESS FUNCTION
# --------------------------------------------------

def decode_yolov11_rknn(outputs, img_shape, model_input_size=(640, 640)):
    """
    outputs: RKNN inference outputs
    img_shape: original image shape (H, W)
    """

    pred = outputs[0]

    # Handle shape variations
    if pred.shape[1] == 84:
        # (1, 84, 8400) → transpose
        pred = pred[0].transpose(1, 0)
    else:
        # (1, 8400, 84)
        pred = pred[0]

    # --------------------------------------------------
    # Split output
    # --------------------------------------------------

    boxes = pred[:, :4]          # xywh
    class_scores = pred[:, 4:]   # class probabilities

    # YOLOv11: apply sigmoid (important for RKNN exports)
    class_scores = sigmoid(class_scores)

    # Get best class per box
    scores = np.max(class_scores, axis=1)
    class_ids = np.argmax(class_scores, axis=1)

    # --------------------------------------------------
    # Filter by confidence
    # --------------------------------------------------

    mask = scores > CONF_THRES

    boxes = boxes[mask]
    scores = scores[mask]
    class_ids = class_ids[mask]

    if len(boxes) == 0:
        return [], [], []

    # --------------------------------------------------
    # Convert xywh → xyxy
    # --------------------------------------------------

    boxes = xywh2xyxy(boxes)

    # --------------------------------------------------
    # Scale boxes back to original image size
    # --------------------------------------------------

    gain_w = img_shape[1] / model_input_size[0]
    gain_h = img_shape[0] / model_input_size[1]

    boxes[:, [0, 2]] *= gain_w
    boxes[:, [1, 3]] *= gain_h

    # Clip
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, img_shape[1])
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, img_shape[0])

    # --------------------------------------------------
    # NMS
    # --------------------------------------------------

    idxs = nms_boxes(boxes, scores, IOU_THRES)

    final_boxes = boxes[idxs]
    final_scores = scores[idxs]
    final_classes = class_ids[idxs]

    return final_boxes, final_scores, final_classes


# --------------------------------------------------
# DRAW FUNCTION
# --------------------------------------------------

def draw_detections(img, boxes, scores, class_ids, class_names):
    for box, score, cls in zip(boxes, scores, class_ids):

        x1, y1, x2, y2 = box.astype(int)

        if cls in class_names:
            cn = class_names[cls]
        else:
            cn = "test"

        label = f"{cn} {score:.2f}"

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.putText(
            img,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    return img