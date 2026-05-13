from ultralytics import YOLO
if __name__=='__main__':
    model=YOLO("runs/segment/train/weights/last.pt")
    print(model)
    breakpoint()
    #model.train(data="seg/data.yaml",epochs=100,batch=8)