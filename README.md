
<p align="center">
  GWMINITEL - Passerelle Internet pour Minitel vers le moniteur Videotex Hydris
</p>

   ____________________________________________________________________________

<p align="center">
Ce logiciel de passerelle vous permettra de vous connecter au serveur Minitel<br>
Hydris avec un vrai Minitel comme utilisé dans les années 80/90'.<br><br>
Par défaut, l'affichage des pages classiques sera en 4800 bauds pour plus de fluidité<br>
et les forums graphiques, eux, seront affichés en 1200 bauds pour une expérience d'origine.<br>
Si vous souhaiter naviguer totalement comme à l'époque en 1200 bauds, il faut utiliser l'option <b>/nospeed</b>.<br><br>
Un câble USB -> DIN Minitel sera utilisé pour la liaison entre le PC et le Minitel<br>(Vous pouvez trouver de très bons câble USB vers Minitel sur eBay chez le sympatique vendeur R-Ecommerce)<br><br>
Grâce à Jean-arthur SILVE qui a créé MiniPavi et à l'excellent émulateur Minitel développé par Frédéric BISSON (un grand merci à eux deux!), une connexion est également possible en cliquant sur le lien suivant:<br>
<a href="https://www.minipavi.fr/emulminitel/indexws.php?url=wss%3A%2F%2Fgalaxy.microtel.fr%3A50123&speed=4800&color=false" title="Galaxy via MiniPavi!">Galaxy via MiniPavi !</a>
</p><br>
  
Options de la ligne de commande : 

| Paramètre                      | Libellé                                                                                            |
|------------------------------|--------------------------------------------------------------------------------------------------------|
| /serial:&lt;ttySerial&gt;    | Driver vers le port série sur lequel est connecté le Minitel (OBLIGATOIRE) *|
| /server:&lt;server&gt;       | Nom DNS ou adresse IPV4 du serveur Hydris sur Internet (défaut galaxy.microtel.fr) |
| /port:&lt;port num&gt;       | Numéro de port du serveur Hydris (défaut 50456) |
| /nospeed                     | Pas de gestion automatique de la vitesse en baud par Hydris. |
| /debug</strong>              | Active le dump de tout ce qui arrive du frontal et du Minitel. |

<br>* Exemples: /dev/tty.usbserial-A5069RR4 sur Mac pour un câble FT232RL du vendeur R-Ecommerce sur eBay ou /dev/ttyUSB0 sous Debian Linux.<br><br>

    Exemple minimal: ./gwminitel /serial:/dev/tty.usbserial-A5069RR4

 
