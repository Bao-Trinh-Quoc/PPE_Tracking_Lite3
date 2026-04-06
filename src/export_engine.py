from ultralytics import YOLO

model = YOLO("best.pt")

print("Exporting best.pt to best.engine... This may take a few minutes.")
model.export(format='engine', half=True, simplify=True)
print("Export complete!")
