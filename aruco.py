import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist  # <--- Nécessaire pour bouger le robot
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np

class ArucoFollowerNode(Node):

    def __init__(self):
        super().__init__('aruco_follower_node')
        
        # --- CONFIGURATION ---
        self.image_topic = '/image_raw/compressed'
        self.cmd_vel_topic = '/cmd_vel'
        
        # Paramètres du robot
        self.max_linear_speed = 0.15  # Vitesse max avant (m/s)
        self.max_angular_speed = 0.8  # Vitesse max rotation (rad/s)
        self.target_dist_pixel = 100  # Taille cible du marqueur (hauteur en px).
                                      # Si < 100 px, le robot avance. Si > 100 px, il s'arrête (il est assez près).
        
        self.kp_angular = 0.003       # Gain proportionnel pour tourner (à ajuster si ça oscille)
        
        self.aruco_dict_type = cv2.aruco.DICT_5X5_250
        # ---------------------

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Abonnement Image
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile)
            
        # Publisher cmd_vel (Pour envoyer les ordres au moteurs)
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Init Aruco
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        self.get_logger().info('Aruco Follower démarré ! Prêt à suivre.')

    def image_callback(self, msg):
        # 1. Décompression
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Erreur image: {e}')
            return

        if cv_image is None: 
            print("image a pb")
            return
        print("Image ok")
        # Récupérer les dimensions de l'image (Hauteur, Largeur)
        h, w, _ = cv_image.shape
        center_x_image = w / 2

        # 2. Détection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        # Message de vitesse par défaut (arrêt)
        twist = Twist()

        if len(corners) > 0:
            print("Aruco detecte")
            # On prend le premier marqueur trouvé pour simplifier
            # corners[0] est de la forme (1, 4, 2) -> 4 coins (x,y)
            marker_corners = corners[0][0]
            
            # --- A. CALCULER LE CENTRE DU MARQUEUR (POUR TOURNER) ---
            # Moyenne des X et des Y des 4 coins
            center_x_marker = np.mean(marker_corners[:, 0])
            center_y_marker = np.mean(marker_corners[:, 1])
            
            # Calcul de l'erreur (différence entre le centre de l'image et le centre du marqueur)
            error_x = center_x_image - center_x_marker
            
            # Application d'un "P-Controller" simple pour la rotation
            # Si le marqueur est à gauche (error positive), on tourne à gauche (z positif)
            twist.angular.z = self.kp_angular * error_x
            
            # Limiter la vitesse de rotation
            twist.angular.z = np.clip(twist.angular.z, -self.max_angular_speed, self.max_angular_speed)

            # --- B. CALCULER LA DISTANCE (POUR AVANCER) ---
            # On utilise la hauteur apparente du marqueur en pixels
            # Coin haut-gauche vs coin bas-gauche
            top_left_y = marker_corners[0, 1]
            bottom_left_y = marker_corners[3, 1]
            marker_height_px = abs(bottom_left_y - top_left_y)

            # Logique d'avancement
            if marker_height_px < self.target_dist_pixel:
                # Si le marqueur est petit (loin), on avance
                # On réduit la vitesse si on tourne beaucoup pour éviter de déraper
                twist.linear.x = self.max_linear_speed * (1.0 - abs(error_x)/(w/2))
                twist.linear.x = max(0.0, twist.linear.x) # Pas de vitesse négative
            else:
                # Si le marqueur est gros (proche), on s'arrête
                twist.linear.x = 0.0
                self.get_logger().info("Cible atteinte (assez proche) !")

            # --- DESSIN ---
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            # Dessiner le centre visé
            cv2.circle(cv_image, (int(center_x_marker), int(center_y_marker)), 5, (0, 255, 0), -1)
            # Dessiner une ligne vers le centre de l'image
            cv2.line(cv_image, (int(center_x_marker), int(center_y_marker)), (int(center_x_image), int(h/2)), (255, 0, 0), 2)

        else:
            # Aucun marqueur : On s'arrête (Sécurité)
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # 3. Envoyer la commande au robot
        self.publisher_.publish(twist)

        # 4. Affichage
        cv2.imshow('Vue Robot (Suivi)', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt propre : envoyer une vitesse nulle avant de couper
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()