#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

rospy.init_node("image_subscriber", anonymous=True)

bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Aplica uma máscara para filtrar apenas pixels verdes
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # lower_green = np.array([40, 50, 50])
        # upper_green = np.array([90, 255, 255])
        lower_green = np.array([40, 75, 75])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Encontra os contornos na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Desenha os contornos na imagem original
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        # Exibe a imagem com os contornos
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)

        # Retorna as coordenadas do contorno mais à esquerda
        if len(contours) > 0:
            # c = max(contours, key=cv2.contourArea)
            # leftmost = tuple(c[c[:,:,0].argmin()][0])
            # print("Coordenadas do objeto verde: ", leftmost) // MOSTRA COORDENADA DO PONTO MAIS A ESQUERDA
            
            # Encontra o contorno com a maior área
            c = max(contours, key=cv2.contourArea)

             # Calcula as propriedades do contorno, incluindo sua área e centroides
            M = cv2.moments(c)

            # Calcula as coordenadas do centro do contorno
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Calcula as dimensões do contorno
            x, y, w, h = cv2.boundingRect(c)

            # Imprime as dimensões do contorno na tela
            print("Dimensões do objeto verde: ({}, {})".format(w, h))

            # Imprime as coordenadas do centro na tela
            # print("Coordenadas do centro do objeto verde: ({}, {})".format(cx, cy))
        
    except CvBridgeError as e:
        print("Frame Dropped: ", e)

rospy.Subscriber("camera/image_raw", Image, image_callback)

rospy.spin()