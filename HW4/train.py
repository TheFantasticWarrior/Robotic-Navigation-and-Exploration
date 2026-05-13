from ultralytics import YOLO
if __name__=='__main__':
    model=YOLO("yolo26n.pt")
    model.train(data="obj/data.yaml",epochs=100,batch=8)