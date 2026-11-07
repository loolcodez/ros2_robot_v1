1) Mitkä paketi(t) asennetaan?
2) Miten paketi(t) asennetaan? Luonko oman paketin itse?
3) Konfigurointi tiedostot. Mitä tiedostoja tarvitsen? Minin ne tulee laittaa?
4) Miten muutan URDF mallia rsp.launch.py. Minulla on jo sellainen missä on
läheisyysanturit kiinni ja se toimii Rviz2 ohjelman kanssa loistavasti.
src/robot_bringup/launch/rsp.launch.py


1) Mitkä paketit asennetaan?

Sinun tarvitsee asentaa vain yksi pääpaketti:
• ros-<distro>-robot-localization (Korvaa <distro> omalla ROS 2 -versiolla, esim. humble, iron tai jazzy).
Tämä paketti sisältää ekf_node-solmun, joka suorittaa sensorifuusion.

2) Miten paketit asennetaan? Luonko oman paketin?

Ei tarvitse luoda uutta pakettia pelkkää EKF:ää varten. Asennat valmiin paketin järjestelmään ja käytät sitä osana robottisi käynnistystiedostoja.
Asennus tapahtuu Ubuntun terminaalista (esimerkkinä ROS 2 Humble):

Install ros-jazzy-robot-localization in robot

sudo apt update
sudo apt install ros-jazzy-robot-localization


3) Konfigurointitiedostot: Mitä tarvitaan ja mihin ne laitetaan?

Tarvitset vain yhden uuden tiedoston: ekf.yaml.
Se kannattaa sijoittaa jo olemassa olevaan omaan pakettiisi (esimerkiksi robot_bringup tai vastaava, jossa säilytät robotin käynnistyskonfiguraatioita) hakemistoon config/.

Esimerkki: config/ekf.yaml

ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true

    # Koordinaatistot
    map_frame: map              # Maailmankoordinaatisto (globaali)
    odom_frame: odom            # EKF:n laskema parannettu odometriakoordinaatisto
    base_link_frame: base_link  # Robotin fyysinen keskipiste
    world_frame: odom           # Koska ei ole GPS:ää, odom on maailmanpiste

    # Pyöräodometria (OdomNode)
    odom0: /odom_wheel
    odom0_config: [false, false, false,  # x,  y,  z
                   false, false, false,  # r,  p,  y
                   true,  false, false,  # vx, vy, vz (Luetaan vain vx eli nopeus eteenpäin)
                   false, false, false,  # v_r, v_p, v_y
                   false, false, false]  # ax, ay, az
    odom0_differential: false

    # IMU-sensori
    imu0: /imu_topic
    imu0_config: [false, false, false,  # x,  y,  z
                  false, false, false,  # r,  p,  y
                  false, false, false,  # vx, vy, vz
                  false, false, true,   # v_r, v_p, v_y (Luetaan v_y eli kääntymisnopeus)
                  true,  false, false]  # ax, ay, az (Otetaan kiihtyvyys eteenpäin)
    imu0_differential: false

Muista lisätä pakettisi CMakeLists.txt tai setup.py tiedostoon sääntö, joka kopioi config/-kansion install/-hakemistoon, jotta ROS löytää sen.

4) Miten muutan URDF-mallia?

URDF-malliisi täytyy lisätä imu_link-liitos (joint) ja -linkki, jotta EKF-solmu tietää tarkalleen, missä kohdassa ja missä asennossa IMU-sensori fyysisesti istuu suhteessa robotin keskipisteeseen (base_link).
Avaa URDF-tiedostosi ja lisää sinne seuraavat rivit (muuta xyz-etäisyyksiä sen mukaan, missä sensori todellisuudessa on):

  <!-- IMU Linkki -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.005"/>
      </geometry>
      <material name="orange"/>
    </visual>
  </link>

  <!-- IMU Liitos base_linkkiin -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <!-- Esimerkki: sensori on 5cm edessä ja 10cm korkeammalla kuin robotin keskipiste -->
    <origin xyz="0.05 0.0 0.10" rpy="0 0 0"/> 
  </joint>

Koska sinulla on jo toimiva rsp.launch.py (Robot State Publisher), se lukee tämän päivitetyn URDF:n automaattisesti ja alkaa julkaista base_link -> imu_link TF-muunnosta Rvizille.

5) Käytetäänkö jotain toista tiedostoa (kuten robot_bringup/launch/...)?

Kyllä, ehdottomasti. rsp.launch.py tiedoston tehtävä on vain lukea URDF ja julkaista robotin passiiviset TF-muunnokset (kuten antureiden paikat). Älä sotke EKF-solmua sen sisään.
Oikea ROS 2 -tapa on luoda ylätason käynnistystiedosto, esimerkiksi bringup.launch.py (tai robot.launch.py) samaan hakemistoon. Tämä tiedosto kutsuu (IncludeLaunchDescription) sinun vanhaa rsp.launch.py -tiedostoasi ja käynnistää samalla EKF-solmun sekä sinun omat pyörä- ja IMU-solmusi.

Esimerkki uudesta src/robot_bringup/launch/bringup.launch.py -tiedostosta:

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup') # Muuta oman pakettisi nimi

    # 1. Kutsutaan vanhaa tuttua RSP-launchia (URDF-mallisi)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py'))
    )

    # 2. Määritellään robot_localization EKF-solmu
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

    # 3. Tähän voit lisätä myös omat kirjoittamasi solmut (Pyöräodometria ja IMU)
    # my_odom_node = Node(package='oma_paketti', executable='odom_node')
    # my_imu_node = Node(package='oma_paketti', executable='imu_node')

    return LaunchDescription([
        rsp_launch,
        ekf_node
        # my_odom_node,
        # my_imu_node
    ])

Kun tämä on valmis, käynnistät koko robotun yhdellä komennolla:
bash
ros2 launch robot_bringup bringup.launch.py

ämä käynnistää URDF-mallin, ottaa raakadatan vastaan antureiltasi, syöttää sen EKF-suodattimelle, ja EKF alkaa julkaista uutta, tietoja yhdistelevää /odom-aihetta sekä dynaamista TF-muunnosta odom -> base_link välille!
Kun saat paketin asennettua, voimme katsoa miten tarkistat Rviz2:ssa, että EKF-solmu toimii oikein ja tekee sille kuuluvan taikansa, eli korjaa pyörien luistaman virheen IMU:n avulla. Haluatko myös apua setup.py / CMakeLists.txt -tiedostojen päivittämisessä tuon uuden config/ekf.yaml-tiedoston osalta?