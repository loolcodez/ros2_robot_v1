"""
Tämä on erinomainen ja teollisuusstandardien mukainen tapa rakentaa robotin arkkitehtuuri. ROS2-maailmassa tällaista solmua kutsutaan usein nimellä Safety Controller tai Safety Node, ja se toimii portinvartijana (multiplexer/filter) ohjauskomentojen ja moottoriohjaimen välillä.
Arkkitehtuuri toimii käytännössä näin:
1. cmd_vel_in (Topic): Ottaa vastaan liikekomentoja joystickiltä, autonomiselta navigoinnilta tai tekoälyltä.
2. range_data (Topic): Kuuntelee VL53L1X-etäisyysanturia.
3. cmd_vel_out (Topic): Lähettää sallitut tai korjatut komennot eteenpäin varsinaiselle moottoriohjaimelle.

Miten kytket tämän osaksi järjestelmää?
Jotta tämä toimii, sinun täytyy muuttaa muiden solmujesi asetuksia:
1. Konfiguroi joystick-ohjelmasi tai tekoälysi julkaisemaan topikkiin cmd_vel_in (aiemman cmd_vel sijaan).
2. Konfiguroi moottoriohjainsolmasi kuuntelemaan topikkia cmd_vel_out.
Miltä tämä arkkitehtuuri vaikuttaa? Jos haluat, voimme seuraavaksi katsoa:
• Miten tämä solmu lisätään proximity_sensor.launch.py-käynnistystiedostoosi, jotta se nousee automaattisesti pystyyn.
• Pitäisikö peruuttaminen (linear.x < 0) sallia silloinkin, kun edessä on este (kuten koodissa nyt on tehty, jotta robotin saa ajettua pois seinästä)?
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- Konfiguraatio ---
        self.SAFETY_DISTANCE_MM = 300  # Pysäytysraja millimetreinä

        # --- Alustukset ---
        self.current_distance_mm = 9999  # Oletuksena tie on vapaa

        # --- Tilaajat (Subscribers) ---
        # Kuunnellaan etäisyysanturia
        self.range_sub = self.create_subscription(Int32, 'range_data', self.range_callback, 10)

        # Kuunnellaan sisääntulevia ohjauskomentoja (joystick, AI jne.)
        self.cmd_vel_in_sub = self.create_subscription(
            Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)

        # --- Julkaisijat (Publishers) ---
        # Lähetetään turvatarkastettu komento moottoriohjaimelle
        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

        self.get_logger().info(f"Safety Node käynnistetty. Turvaraja: {self.SAFETY_DISTANCE_MM} mm.")

    def range_callback(self, msg):
        """Päivittää tuoreimman etäisyystiedon anturilta."""
        self.current_distance_mm = msg.data

    def cmd_vel_callback(self, msg):
        """Tarkastaa sisääntulevan ajokomennon turvallisuuden."""
        safe_msg = Twist()

        # Kopioidaan pyörähdysnopeus (kääntymistä ei yleensä tarvitse estää)
        safe_msg.angular = msg.angular

        # Tarkastetaan eteenpäin suuntautuva liike (linear.x > 0)
        if msg.linear.x > 0 and self.current_distance_mm < self.SAFETY_DISTANCE_MM:
            # ESTE EDESSÄ: Pakotetaan eteenpäin vievä nopeus nollaksi
            safe_msg.linear.x = 0.0
            self.get_logger().warn(
                f"HÄTÄPYSÄYTYS! Este edessä: {self.current_distance_mm} mm. Komento estetty.",
                throttle_duration_sec=1.0  # Tulostetaan lokiin max kerran sekunnissa
            )
        else:
            # Tie on vapaa tai robotti ajaa peruuttamalla (linear.x <= 0)
            safe_msg.linear.x = msg.linear.x

        # Julkaistaan suodatettu komento moottoreille
        self.cmd_vel_out_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
