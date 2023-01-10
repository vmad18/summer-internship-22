from pyzbar.pyzbar import decode
import cv2 as cv
import numpy as np


def scan_qr_code(frame)-> list:
    gray = cv.cvtColor(frame, 0) 
    qr = decode(gray) 

    outs:list = []

    if len(qr) == 0: 
        print("no qr codes found")
        return outs


    for code in qr: 
        data = code.data.decode("utf-8")
        outs.append(str(data))
    return outs


def main()-> None:
    img = cv.imread("file.png")

    qr_codes:list = scan_qr_code(img)
    for string in qr_codes:
        print(string)

if __name__ == "__main__":
    main()