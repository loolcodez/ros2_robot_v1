

# rmw_zenoh_cpp



1. Asenna rmw_zenoh_cpp

Päivitä pakettilistat ja asenna Zenoh RMW -paketti robotin terminaaliin:

sudo apt update
sudo apt install ros-jazzy-rmw-zenoh-cpp


2. Aseta ympäristömuuttujat

Jotta ROS 2 käyttää Zenohia oletuksena kaikkien noodien välillä, lisää RMW_IMPLEMENTATION pysyvästi ~/.bashrc-tiedostoon:

echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
source ~/.bashrc

(Varmista myös, että source /opt/ros/jazzy/setup.bash on ladattu ennen tätä riviä).


3. Määritä Zenoh Router (rmw_zenohd) verkkoa varten

Jotta ulkopuoliset laitteet (Flutter-tabletti ja AI-palvelin) saavat yhteyden robottiin Wi-Fi-verkon yli, Zenoh-reitittimen on kuunneltava kaikkia verkkoliitäntöjä (0.0.0.0) oletusportissa 7447.

3.1. Kopioi oletuskonfiguraatio muokattavaksi:

mkdir -p ~/zenoh_config
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 ~/zenoh_config/router_config.json5

3.2. Tarkista kuunteluosoite tiedostosta ~/zenoh_config/router_config.json5:
Varmista, että listen-lohkosta löytyy tcp/0.0.0.0:7447 (useimmissa Jazzy-versioissa tämä on jo oletuksena tcp/[::]:7447, joka kuuntelee sekä IPv4- että IPv6-osoitteita).

3.3. Laita jaettu muisti (Shared Memory) päälle robotin sisäistä nopeutta varten:
Voit ottaa sen käyttöön joko konfiguraatiotiedostosta tai ympäristömuuttujalla:

export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'



4. Testaa asennus

Avaa kolme terminaalia robotissa ja testaa, että viestit kulkevat Zenohin läpi:
• Terminaali 1 (Käynnistä Zenoh-reititin):bash
ros2 run rmw_zenoh_cpp rmw_zenohd
Käytä koodia harkiten.
• Terminaali 2 (Kuuntelija):bash
ros2 run demo_nodes_cpp listener
Käytä koodia harkiten.
• Terminaali 3 (Lähettäjä):bash
ros2 run demo_nodes_cpp talker
Käytä koodia harkiten.
Jos näet I heard: [Hello World: ...] -viestit Terminaalissa 2, asennus ja reititys toimivat.

5. Käynnistä reititin automaattisesti robotin bootissa (Systemd)

Jotta sinun ei tarvitse käynnistää rmw_zenohd-prosessia käsin joka kerta, luo sille taustapalvelu:
bash
sudo nano /etc/systemd/system/zenoh-router.service
Käytä koodia harkiten.
Liitä tiedostoon seuraava sisältö (korvaa ubuntu robotin omalla käyttäjänimellä):
ini
[Unit]
Description=ROS 2 Zenoh Router
After=network.target

[Service]
Type=simple
User=ubuntu
Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
Käytä koodia harkiten.
Ota palvelu käyttöön ja käynnistä se heti:
bash
sudo systemctl daemon-reload
sudo systemctl enable zenoh-router.service
sudo systemctl start zenoh-router.service

Käytä koodia harkiten.
Kun olet saanut reitittimen pyörimään, haluatko seuraavaksi konfiguraation client-noodeille (esim. AI-palvelimen tai tabletin yhdistäminen robotin IP-osoitteeseen) vai koodipohjan safety-noodille, joka ottaa vastaan 8 tavun joystick-viestejä?