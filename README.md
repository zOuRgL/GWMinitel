
<p align="center">
  GWMINITEL - Passerelle Internet pour Minitel vers le moniteur Videotex Hydris
</p>

   ____________________________________________________________________________

Un câble USB -> DIN sera utilisé pour la liaison entre le PC et le Minitel

Options de la ligne de commande : 

 <strong>/serial:&lt;ttySerial&gt;</strong> : 
     Driver vers le port série sur lequel est connecté le Minitel (OBLIGATOIRE).
     Exemples: /dev/tty.usbserial-A5069RR4 sur Mac pour un câble FT232RL 
              du vendeur R-Ecommerce sur eBay.
              /dev/ttyUSB0 sous Debian Linux pour le même câble.

 <strong>/server:&lt;server&gt;</strong>    : Nom DNS ou adresse IPV4 du serveur Hydris sur Internet (défaut galaxy.microtel.fr)

 <strong>/port:&lt;port num&gt;</strong>    : Numéro de port du serveur Hydris (défaut 50456)

 <strong>/nospeed</strong>            : Pas de gestion automatique de la vitesse en baud par Hydris.
                       (utile en cas de problèmes)

 <strong>/debug</strong>              : Active le dump de tout ce qui arrive du frontal et du Minitel.


    Exemple: ./gwminitel /serial:/dev/tty.usbserial-A5069RR4

 
