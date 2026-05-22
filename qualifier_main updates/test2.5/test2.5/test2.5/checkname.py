from ultralytics import YOLO
model = YOLO("best.pt")
print(model.names)  # Check exact class names your model uses
