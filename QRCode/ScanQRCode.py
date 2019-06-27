from pyzbar import pyzbar
#import argparse
import cv2
#import qrcode
#from matplotlib import pyplot as plt
#from imutils.video import VideoStream
#import imutils
import time
print('[INFO]starting video stream...')
cap = cv2.VideoCapture(0)
time.sleep(2.0)
while True:
    f,frame = cap.read()
    barcodes = pyzbar.decode(frame)  # 解析二维码
    cv2.imshow('image', frame)  # 显示二维码
    cv2.waitKey(0)  # 等待按键
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect  # 获取位置
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 绘制
        barcodeData = barcode.data.decode('utf-8')
        barcodeType = barcode.type
        text = "{}({})".format(barcodeData, barcodeType)
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow('image', frame)  # 显示二维码
        print(barcodeData)  # 输出二维码内信息

        cv2.waitKey(0)  # 等待按键
cap.release()
#img  = qrcode.make('Denny First QRCode')#生成二维码
#with open('./data/test.png','wb') as f:#创建文件
#    img.save(f)#保存文件
#img = cv2.imread('./data/test.png')#识别二维码
#cv2.imshow('image',img)#显示二维码
#cv2.waitKey(0)#等待按键

