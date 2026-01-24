
                                            VERSION DE TEST ! 


 GwMinitel - Passerelle Minitel vers le serveur Hydris via Internet v0.30

Options de la ligne de commande : 

 /serial:<ttySerial> : 
     Driver vers le port série sur lequel est connecté le Minitel. (OBLIGATOIRE)
     Exemples: /dev/tty.usbserial-A5069RR4 sur Mac pour un câble FT232RL 
               du vendeur R-Ecommerce sur eBay.
               /dev/ttyUSB0 sous Debian Linux pour le même câble.

 /server:<ip server> : Nom DNS ou adresse IPV4 du serveur Hydris sur Internet.

 /port:<port num>    : Numéro de port du serveur Hydris (défaut 50456)

 /nospeed            : Pas de gestion automatique de la vitesse en baud par Hydris.
                       (utile en cas d'utilisation d'un MINITEL 1 ou bien en cas de problèmes
                        lors de saisies de données sans attendre la fin d'affichage)

 /debug              : Active le dump de tout ce qui arrive du frontal et du Minitel.

 
