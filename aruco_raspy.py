import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np


class ArucoFollowerNode(Node):

    def __init__(self):
        super().__init__('aruco_follower_node')

        # --- CONFIGURATION ---
        self.image_topic = '/camera/image_raw/compressed'
        self.cmd_vel_topic = '/cmd_vel'

        # Paramètres du robot
        self.max_linear_speed = 0.15   # m/s
        self.max_angular_speed = 0.8   # rad/s
        self.target_dist_pixel = 100   # hauteur cible du marqueur en px
        self.kp_angular = 0.003        # gain P pour la rotation

        self.aruco_dict_type = cv2.aruco.DICT_5X5_250
        # ---------------------

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Abonnement Image (CompressedImage)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        # Publisher cmd_vel
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --- Init ArUco (ancienne API OpenCV) ---
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        self.get_logger().info('Aruco Follower démarré ! Prêt à suivre.')

    def image_callback(self, msg):
        # 1) Décompression
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Erreur image: {e}')
            return

        if cv_image is None:
            self.get_logger().warn("Image décodée = None")
            return

        # Dimensions image
        h, w, _ = cv_image.shape
        center_x_image = w / 2.0

        # 2) Détection ArUco (ancienne API)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.arucoDict,
            parameters=self.arucoParams
        )

        # Commande par défaut : stop
        twist = Twist()

        if ids is not None and len(corners) > 0:
            # On prend le premier marqueur détecté
            marker_corners = corners[0][0]  # shape (4,2)

            # A) Centre du marqueur (pour tourner)
            center_x_marker = float(np.mean(marker_corners[:, 0]))
            center_y_marker = float(np.mean(marker_corners[:, 1]))

            error_x = center_x_image - center_x_marker

            twist.angular.z = self.kp_angular * error_x
            twist.angular.z = float(np.clip(twist.angular.z, -self.max_angular_speed, self.max_angular_speed))

            # B) Taille apparente (pour avancer)
            # corners: [top-left, top-right, bottom-right, bottom-left]
            top_left_y = marker_corners[0, 1]
            bottom_left_y = marker_corners[3, 1]
            marker_height_px = abs(bottom_left_y - top_left_y)

            if marker_height_px < self.target_dist_pixel:
                # vitesse réduite si grosse erreur de centrage
                twist.linear.x = self.max_linear_speed * (1.0 - abs(error_x) / (w / 2.0))
                twist.linear.x = max(0.0, float(twist.linear.x))
            else:
                twist.linear.x = 0.0
                self.get_logger().info("Cible atteinte (assez proche) !")

            # Dessin debug
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.circle(cv_image, (int(center_x_marker), int(center_y_marker)), 5, (0, 255, 0), -1)
            cv2.line(
                cv_image,
                (int(center_x_marker), int(center_y_marker)),
                (int(center_x_image), int(h / 2)),
                (255, 0, 0),
                2
            )

            # Log utile (optionnel)
            # self.get_logger().info(f"IDs détectés : {ids.flatten().tolist()} | height_px={marker_height_px:.1f}")

        else:
            # Aucun marqueur : stop (sécurité)
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # 3) Envoi commande
        self.publisher_.publish(twist)

        # 4) Affichage
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
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
