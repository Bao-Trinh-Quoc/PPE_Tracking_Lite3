from ultralytics import YOLO

# Load the PyTorch model
model = YOLO('best.pt')

# Export the model to TensorRT engine format
# Note: This will generate 'best.engine' in the same directory
print("Exporting best.pt to best.engine... This may take a few minutes.")
model.export(format='engine', half=True, simplify=True)
print("Export complete!")
