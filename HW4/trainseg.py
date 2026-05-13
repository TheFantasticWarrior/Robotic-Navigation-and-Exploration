from ultralytics import YOLO
if __name__=='__main__':
    model=YOLO("yolo26n-seg.pt")
    model.train(data="seg/data.yaml",epochs=100,batch=8)