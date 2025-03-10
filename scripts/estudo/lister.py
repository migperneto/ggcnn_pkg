#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def lister():
    #Inicializar o nó
    rospy.init_node('Node_1', anonymous=True)

    #Definir o publisher
    chatter_pub = rospy.Publisher('/chatter', String, queue_size=100)

    #Definir a taxa de atualização
    rate = rospy.Rate(10)

    n = 0

    while not rospy.is_shutdown():
        #Texto da mensagem
        texto = f'Contagem: {n}'

        #Mostrar mensagem no log
        rospy.loginfo(texto)

        #Publicar mensagem
        chatter_pub.publish(texto)

        #Pausar
        rate.sleep()

        #Contagem
        n+=1

if __name__ == '__main__':
    try:
        lister()
    except rospy.ROSInterruptException():
        pass

