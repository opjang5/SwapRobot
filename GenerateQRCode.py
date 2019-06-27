import qrcode
import cv2
img  = qrcode.make('Window')#生成二维码
with open('./data/Window.png','wb') as f:#创建文件
    img.save(f)#保存文件
#img = cv2.imread('./data/Charge.png')#识别二维码
#cv2.imshow('image',img)#显示二维码
#cv2.waitKey(0)#等待按键
