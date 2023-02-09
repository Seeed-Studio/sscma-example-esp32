from serial import Serial
from serial.tools.list_ports import comports
import time
import re
import base64
from PIL import Image
from io import BytesIO
import numpy as np
import cv2


def base64img(width, height, channel, img):

    format = 'RGB'

    if channel == 3:
        format = 'RGB'
    elif channel == 1:
        format = 'L'

    try:
        imgdata = base64.b64decode(img)
        img = Image.frombytes(format, (width, height), imgdata)
        img = np.array(img)
        return img
    except:
        return None


if __name__ == '__main__':

    ser = Serial('COM6', 2000000, timeout=2000)

    # text = 'AT+RUNIMPULSEDEBUG=y\r\n'
    # result = ser.write(text.encode("utf8"))

    data = ''
    width = 0
    height = 0
    channel = 0

    while True:
        rev_num = ser.inWaiting()
        if rev_num:
            s = ser.read(rev_num)
            data += str(s, encoding='utf-8')

            if data.find('End output') > -1:
                currentMsg = data.split('End output')[0]
                img = None

                line = currentMsg.split('\n')

                for i in range(len(line)):
                    if line[i].find('Format: ') > -1:
                        fmt = eval(line[i].split('Format: ')
                                   [1].replace('\r', ''))
                        width = fmt['width']
                        height = fmt['height']
                        channel = fmt['channels']

                    if line[i].find('Framebuffer: ') > -1:
                        fb = line[i].split('Framebuffer: ')[
                            1].replace('\r', '')
                        img = base64img(width, height, channel, fb)

                    if line[i].find('Predictions ') > -1:
                        print(line[i].split('Predictions ')
                              [1].replace('\r', ''))

                    if line[i].find('    ') > -1:
                        print(line[i])
                        if line[i].find('No objects found') <= -1:
                            label = line[i].split('(')[0].strip()
                            c = line[i].split('(')[1].split(')')[0]
                            x = int(line[i].split('x: ')[1].split(',')[0])
                            y = int(line[i].split('y: ')[1].split(',')[0])
                            w = int(line[i].split('width: ')[1].split(',')[0])
                            h = int(line[i].split('height: ')
                                    [1].split(' ]')[0])
                            print(label, c, x, y, w, h)
                            cv2.circle(img, (x, y), w, (0, 0, 255), 1)
                            cv2.putText(img, label + ':' + c,
                                        (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

                if type(img) != type(None):
                    img = cv2.resize(img, dsize=(240, 240))
                    cv2.imshow('img', img)
                    cv2.waitKey(1)
                    img = None

                data = data.split('End output')[1]

            if data.find('Begin output') > -1:
                data = data.split('Begin output')[1]
