
/*

 * Copyright (C) 2026, Eric Joseph MANGONI
 *
 * Ce programme est libre : vous pouvez le redistribuer et/ou le modifier
 * selon les termes de la GNU General Public License version 3 publiée 
 * par la Free Software Foundation.
 *
 * Ce programme est distribué dans l'espoir qu'il sera utile,
 * mais SANS AUCUNE GARANTIE.


    Passerelle de connexion Mac/Linux vers frontal Hydris pour un Minitel local

                                                                       '2026


    Compilation classique avec : 
        Mac/Linux : gcc gwminitel.c -o gwminitel 

        Compilation pour x86 à partir d'un Mac Apple Silicon : 
        clang -arch x86_64  gwminitel.c -o gwminitel

*/

/*

 Memento codes Videotex 

 ESC 39 (9) XX            ! Commandes protocole a un argument (PRO1).	  
 ESC 3A (:) XX XX         ! Commandes protocole a deux arguments (PRO2).   
 ESC 3B (;) XX XX XX      ! Commandes protocole a trois arguments (PRO3). 

*/


#include <ctype.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <termios.h>
#include <sys/time.h>
#include <netdb.h>

//
#define VERSION       "0.30"
// 

#define FALSE              0
#define TRUE               1

#define STX             0x02    // Gestion trames
#define ETX             0x03
#define TYPE            0x04
#define CV              0x05
#define DATA            0x06

#define PTYPE_LOGIN       65    // Types de paquet pour communication avec Hydris
#define PTYPE_LOGOUT      66
#define PTYPE_DATA        67
#define PTYPE_PING        68
#define PTYPE_SYSTEM      69

#define FNCT_MNTL       0x13    // Premiere partie touche de fnct minitel.

#define ENVOI           0x41
#define RETOUR          0x42
#define REPET           0x43
#define GUIDE           0x44
#define ANNUL           0x45
#define SOMMAIRE        0x46
#define CORRECT         0x47
#define SUITE           0x48
#define CNXFIN          0x49

#define MINITEL1        0x00    // Différents types de Minitel pour détection
#define MINITEL1COLOR   0x01
#define MINITEL1DIAL    0x02
#define MINITEL1B       0x03
#define MINITEL2        0x04
#define MINITEL5        0x04
#define MINITEL10       0x06
#define MINITEL10B      0x07
#define MINITEL12       0x08
#define MINITELMAGIS    0x09
#define MINITEL_UNKNOWN 0x10

#define UPDATE_VERSION   252    // Etat du service
#define SERVER_CLOSED    253
#define CNX_REFUSED      254

#define BUFFERSIZE      8192    // Taille du buffer d'entrée/sortie pour la communication avec Hydris
#define TXBUFFERSIZE    8192

// Variables & structures
typedef struct {
    unsigned char buffer[TXBUFFERSIZE+1];
    int head;  // index d'écriture
    int tail;  // index de lecture
    int count; // nombre de caractères en attente
} tx_buffer_t;

static struct termios Old_term;
char                  Dump;
tx_buffer_t           COM_TXBuffer; // Buffer série TX 
//


/*
  
            Gestion du buffer série TX - Merci à ChatGPT ;)  

            Utilisation d'un buffer logiciel car sinon lors des changements de vitesse, le Minitel perd un peu les pédales
            et ne synchronise plus avec le soft. Cela arrive dans le cas où l'utilisateur tape au clavier lorsque une page est
            toujours en cours d'affichage. Avec le buffer logiciel synchronisé avec la vitesse de transmission en cours, le
            problème ne se produit plus car il peut être vidé proprement avant tout changement de débit. 

*/

// Initialisation de démarrage
void tx_buffer_init(tx_buffer_t *txbuf)
{
    txbuf->head = 0;
    txbuf->tail = 0;
    txbuf->count = 0;
}

// Ajout un caractère dans le buffer
int tx_buffer_push(tx_buffer_t *txbuf, unsigned char c)
{
    if (txbuf->count >= TXBUFFERSIZE) return -1; // buffer plein

    txbuf->buffer[txbuf->head] = c;
    txbuf->head = (txbuf->head + 1) % TXBUFFERSIZE;
    txbuf->count++;
    return 0;
}

// Vide le buffer
void tx_buffer_clear(tx_buffer_t *txbuf)
{
    txbuf->head = 0;
    txbuf->tail = 0;
    txbuf->count = 0;
}

// Combien de caractères restant dans le buffer ?
int tx_buffer_count(tx_buffer_t *txbuf)
{
    return txbuf->count;
}

// Est ce que le port série est prêt à recevoir un nouveau caractère à envoyer ?
int tx_can_send_now(int fd)
{
    int outq;
    if (ioctl(fd, TIOCOUTQ, &outq) == -1)
        return 0;

    return (outq == 0);
}

// Attendre le temps que le buffer se vide 
void tx_buffer_purge(int fd, tx_buffer_t *txbuf, int baudrate)
{
    // Calculer le temps total nécessaire pour vider le buffer
    int chars_remaining = txbuf->count;
    if (chars_remaining <= 0) {
        tx_buffer_clear(txbuf);
        return;
    }

    double time_sec = (chars_remaining * 10.0) / baudrate;

    // Vider le buffer logiciel immédiatement
    tx_buffer_clear(txbuf);

    // Attente pour laisser le temps au port série d'envoyer les caractères restants
    struct timespec ts;
    ts.tv_sec = (time_t)time_sec;
    ts.tv_nsec = (long)((time_sec - ts.tv_sec) * 1e9);

    nanosleep(&ts, NULL);
}

/* Envoi non bloquant du buffer TX pour Minitel 7E1 (9 bits par caractère) */
void tx_buffer_send(int fd, tx_buffer_t *txbuf, int baudrate)
{
    static struct timespec last_send = {0, 0};
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (!tx_can_send_now(fd))
       return; // On attend que le port soit prêt

    // Temps écoulé en nanosecondes depuis le dernier envoi
    long elapsed_ns = (now.tv_sec - last_send.tv_sec) * 1000000000L
                    + (now.tv_nsec - last_send.tv_nsec);

    // Nombre de bits transmis par ns
    double bits_per_ns = baudrate / 1e9;
    double bits_elapsed = bits_per_ns * elapsed_ns;

    // 7E1 = 9 bits par caractère
    int chars_to_send = (int)(bits_elapsed / 9.0);
    if (chars_to_send < 1) return;

    for (int i = 0; i < chars_to_send && txbuf->count > 0; i++)
    {
        unsigned char c = txbuf->buffer[txbuf->tail];
        write(fd, &c, 1);  // envoi non bloquant

        txbuf->tail = (txbuf->tail + 1) % TXBUFFERSIZE;
        txbuf->count--;
    }

    last_send = now;
}

/* Fonction bloquante : vide tout le buffer TX en envoyant les caractères */
void tx_buffer_flush(int fd, tx_buffer_t *txbuf, int baudrate)
{
    while (txbuf->count > 0)
    {
        // Vider quelques caractères selon le temps écoulé
        tx_buffer_send(fd, txbuf, baudrate);

        // Calculer temps à attendre avant le prochain caractère
        // 10 bits par caractère (start + data + parity + stop)
        //double char_time_sec = 10.0 / baudrate;
        double char_time_sec = 9.0 / baudrate;

        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = (long)(char_time_sec * 1e9);

        nanosleep(&ts, NULL);
    }

    // Optionnel : s'assurer que le port série a fini de tout transmettre
    tcdrain(fd); // Bloquant jusqu'à ce que tout soit physiquement envoyé
}

//      Fin des fonctions de gestion du buffer série TX


// Fonctions diverses

void enable_raw_mode() {
    struct termios new_term;
    tcgetattr(STDIN_FILENO, &Old_term);
    new_term = Old_term;
    new_term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
}

void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &Old_term);
}

void delay_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// Ip de la forme (char *)x.x.x.x à 4 (uint)octets xxxx
int ip_compress(const char *ip_str, uint8_t out[4])
{
    return inet_pton(AF_INET, ip_str, out) == 1 ? 0 : -1;
}

// Ip de la forme 4 octets xxxx à (char *)x.x.x.x 
int ip_decompress(const uint8_t in[4], char *ip_str, size_t len)
{
    return inet_ntop(AF_INET, in, ip_str, len) ? 0 : -1;
}

// Quelle est l'adresse externe du client ?
int get_external_ip(char *ip, size_t ip_len)
{
    struct addrinfo hints = {0}, *res;
    int sock = -1;
    char buffer[1024];
    char *body;
    ssize_t n;
    size_t total = 0;

    if (!ip || ip_len < 8)
        return -1;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo("api.ipify.org", "80", &hints, &res) != 0)
        return -1;

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0)
        goto error;

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0)
        goto error;

    const char *request =
        "GET / HTTP/1.1\r\n"
        "Host: api.ipify.org\r\n"
        "Connection: close\r\n\r\n";

    if (send(sock, request, strlen(request), 0) < 0)
        goto error;

    /* Lecture complète */
    while ((n = read(sock, buffer + total,
                     sizeof(buffer) - total - 1)) > 0)
    {
        total += n;
        if (total >= sizeof(buffer) - 1)
            break;
    }
    buffer[total] = '\0';

    body = strstr(buffer, "\r\n\r\n");
    if (!body)
        goto error;
    body += 4;

    size_t i = 0;
    while (*body && i < ip_len - 1) {
        if (isdigit((unsigned char)*body) || *body == '.')
            ip[i++] = *body;
        else
            break;
        body++;
    }
    ip[i] = '\0';

    close(sock);
    freeaddrinfo(res);
    return 0;

error:
    if (sock >= 0) close(sock);
    freeaddrinfo(res);
    return -1;
}

/* Ouvre et configure le port série 1200,E,7,1 */
int serial_open(const char *device)
{
    int fd_serie;

    fd_serie = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_serie == -1) {
        perror("Erreur ouverture port série");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd_serie, &tty) != 0) {
        perror("Erreur tcgetattr");
        close(fd_serie);
        return -1;
    }

    // Configuration : 1200 bauds, 7 bits, Even parity, 1 stop bit
    cfsetospeed(&tty, B1200);
    cfsetispeed(&tty, B1200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;  // 7 bits
    tty.c_cflag |= PARENB;                       // Parity activée
    tty.c_cflag &= ~PARODD;                      // Even parity
    tty.c_cflag |= CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // Pas de flow control hardware
    tty.c_cflag |= CLOCAL | CREAD;               // Local, lecture activée

    tty.c_lflag &= ~ICANON;                      // Mode brut
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // Pas de flow control software
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;                       // Pas de traitement sortie

    tty.c_cc[VMIN] = 1;                          // 1 caractère minimum
    tty.c_cc[VTIME] = 0;                         // Pas de timeout

    if (tcsetattr(fd_serie, TCSANOW, &tty) != 0) {
        perror("Erreur tcsetattr");
        close(fd_serie);
        return -1;
    }

    tcflush(fd_serie, TCIOFLUSH);  // Vider les buffers

    return fd_serie;
}

// Modif vitesse de transmission de la série
int serial_set_baudrate(int fd, speed_t speed)
{
    struct termios tio;

    /* Lire la configuration actuelle */
    if (tcgetattr(fd, &tio) != 0) {
        perror("tcgetattr");
        return -1;
    }

    /* Modifier uniquement la vitesse */
    if (cfsetispeed(&tio, speed) != 0) {
        perror("cfsetispeed");
        return -1;
    }

    if (cfsetospeed(&tio, speed) != 0) {
        perror("cfsetospeed");
        return -1;
    }

    /* Appliquer sans modifier le reste */
    //if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) {
        perror("tcsetattr");
        return -1;
    }

    tcflush(fd, TCIOFLUSH);  ///  
    return 0;
}

/* Envoie un caractère sur la série */
int serial_write_char(int fd, unsigned char c)
{
    tx_buffer_push(&COM_TXBuffer, c);
    return 0;
}

/* Envoie une chaîne sur la série */
int serial_write_string(int fd, const char *str, int len)
{
 for( int i=0; i<len; i++ )
    tx_buffer_push(&COM_TXBuffer, str[i] );
 return FALSE;
}

// Résolution du nom DNS du frontal (Retourne 0 en cas de succès, 1 en cas d'erreur)
int DNStoIpv4(char *dnsName, char *ipAddress) {
    struct addrinfo hints, *res, *p;
    int status;
    
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;       // IPv4 uniquement
    hints.ai_socktype = SOCK_STREAM;
    
    status = getaddrinfo(dnsName, NULL, &hints, &res);
    if (status != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(status));
        return 1;
    }
    
    // Récupérer la première adresse IPv4
    struct sockaddr_in *ipv4 = (struct sockaddr_in *)res->ai_addr;
    inet_ntop(AF_INET, &(ipv4->sin_addr), ipAddress, INET_ADDRSTRLEN);
    
    freeaddrinfo(res);
    return 0;
}



/*


                    FONCTION PRINCIPALE


*/

int main(int argc, char *argv[]) {

    char *tampon,minitelModel[4],serialPath[128],hydrisAddr[128],externalIP[32];
    int  instate = 0,instateServer = 0,speedMode,currentSpeed,serverPort,typeMinitel = MINITEL_UNKNOWN,hydrisConnected = 0;
    time_t tDeb,tNow;
    uint8_t bufIP[4];

    char MinitelModelTxt[10][16] = { "Minitel 1", "Minitel 1C", "Minitel 1D", "Minitel 1B",
                                     "Minitel 2", "Minitel 5", "Minitel 10",   "Minitel 10B",
                                     "Minitel Magis", "Minitel ?" };

    enable_raw_mode();
    atexit(disable_raw_mode);

    Dump = FALSE;
    speedMode = TRUE;

    if ( argc < 2 || argc >= 2 && !strcmp( argv[1], "/?" )){
        printf("\n GwMinitel - Passerelle Minitel vers le serveur Hydris via Internet v%s\n\n", VERSION);

        printf(" /serial:<ttySerial> : Driver vers le port série sur lequel est connecté le Minitel. (OBLIGATOIRE)\n");
        printf("                       Exemples: /dev/tty.usbserial-A5069RR4 sur Mac pour un câble FT232RL du vendeur R-Ecommerce sur eBay.\n");
        printf("                                 /dev/ttyUSB0 sous Debian Linux pour le même câble.\n");

        printf("\n /server:<ip server> : Nom DNS ou adresse IPV4 du serveur Hydris sur Internet.\n");

        printf("\n /port:<port num>    : Numéro de port du serveur Hydris (défaut 50456)\n");

        printf("\n /nospeed            : Pas de gestion automatique de la vitesse en baud par Hydris.\n" );
        printf("                       (utile en cas d'utilisation d'un MINITEL 1 ou bien en cas de problèmes\n");
        printf("                        lors de saisies de données sans attendre la fin d'affichage)\n");

        printf("\n /debug              : Active le dump de tout ce qui arrive du frontal et du Minitel.\n" );

        printf("\n\n");
        return TRUE;
    }
    serialPath[0]='\0';                             // tty de la liaison série
    strcpy( hydrisAddr, "galaxy.microtel.fr" );     // Adresse par défaut du frontal vers Hydris
    serverPort = 50456;                             // Port par défaut 

    if ( argc > 1 ){
       for( int i = 1; i<argc; i++ ){
          //printf("%d. Argument %s\n",argc, argv[i]);

          if ( !strncmp( argv[1], "/serial:", 8 ) && strlen(argv[1]) > 9 ) 
             strcpy( serialPath, argv[1]+8 );
        
          if ( !strncmp( argv[i], "/server:", 8 ) && strlen(argv[i]) > 9 ) 
             strcpy( hydrisAddr, argv[i]+8 );

          if ( !strncmp( argv[i], "/port:", 6 ) && strlen(argv[i]) > 7 ) 
             serverPort = atoi( argv[i]+6 ); 

          if ( !strcmp( argv[i], "/nospeed" ) ) 
             speedMode = FALSE;
          
          if ( !strcmp( argv[i], "/debug" ) ) 
             Dump = TRUE;

       }

    }

    // Récupère l'IP externe du client ou 1.1.1.1 si impossible
    if ( get_external_ip(externalIP, sizeof(externalIP)) )
       strcpy( externalIP, "1.1.1.1");

    tampon = malloc( BUFFERSIZE+1 );
    unsigned long total_chars = 0;
    int cnxAccepted = 0,cxnRefused = 0;
    
    // Résolution du nom DNS pour la connexion au frontal
    strcpy( tampon, "NONE" );
    DNStoIpv4( hydrisAddr, tampon );
    strcpy( hydrisAddr, tampon );

    // Initialisation du buffer TX série
    tx_buffer_init(&COM_TXBuffer);

    // Ouverture du port série pour le Minitel
    int fdserial = serial_open(serialPath);
    if (fdserial < 0) {
          printf("Impossible d'ouvrir %s\n", serialPath);
          return 1;
    }

    system( "clear");   // Efface l'écran
    printf("\n\n→ Client Minitel pour frontal Hydris v%s\n\n", VERSION);

    // --- Socket serveur (Frontal) ---
    int server_sock = -1;       // Pas de connexion pour l'instant

    printf("→ Adresse du serveur Hydris %s:%d\n", hydrisAddr, serverPort);
    printf("→ Port série ouvert : %s\n", serialPath);
   
    // Changement de la vitesse du Minitel à 4800bds pour plus de fluidité. 
    currentSpeed = 1200;
    if ( speedMode ){
       currentSpeed = 4800;
       serial_write_string( fdserial, "\x1b:kv", 4 );  // Minitel en 4800bds
       tx_buffer_flush(fdserial, &COM_TXBuffer, 1200 );
       delay_ms(400);
       serial_set_baudrate(fdserial, B4800 );          
    }
    
    // Affiche la page d'accueil
    sprintf( tampon, "\x1b\x3a\x32\x7E\x14\x1F\x30\x30\x18\x0C\x1b\x42________________________________________\x1BQ \x18\x0A\x0D\x1BU \x18\x1BM"
                     "\x1B@Passerelle Minitel vers serveur Hydris\x1BM\x1F\x44\x41\x1B\x42~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\x1FHL\x1B]"
                     "\x1B\x42 Sommaire \x1B\\ Connexion\x1FJF\x1B]\x1B\x45 Shift + Cnx/Fin \x1B\\\x1b\x42 D\031Beconnexion\x0a\x0a\x0a\x0d"
                     "\x1F\x56\x46\x1b\x42 2026, Minitel is still alive!\x1B@\x1B;\x60XR\x1B:iE" );
    serial_write_string( fdserial, tampon, strlen(tampon) );
    
    if ( !speedMode )
        printf("→ /nospeed activé! Pas de gestion de la vitesse dans le site.\n");
 
    if ( Dump ) 
       printf("→ DUMP Actif.\n");
    else
       printf("→ DUMP désactivé.\n");   

    printf( "\n  [ ECHAP ] Pour deconnecter et quitter.\n\n\n" );

    tDeb = time(NULL); // Gestion du timeout pour la déconnexion automatique

    // Go ! 
    while (1) {

        tx_buffer_send(fdserial, &COM_TXBuffer, currentSpeed);

        fd_set read_fds; FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO,&read_fds); 
        FD_SET(server_sock,&read_fds); 
        FD_SET(fdserial,&read_fds); 
        int maxfd = server_sock>fdserial?server_sock:fdserial;

        struct timeval tv = {0, 1000}; // timeout 1ms
        int activity = select(maxfd+1, &read_fds, NULL, NULL, &tv);

        if (activity > 0) {

            struct sockaddr_in addr;
            socklen_t addrlen = sizeof(addr);    
                    
            //
            //       --- Lecture depuis le port série ---
            //

            if (fdserial >= 0 && FD_ISSET(fdserial, &read_fds)) {
                unsigned char buf;
                int cv;
                ssize_t n = read(fdserial, &buf, 1);
                if (n > 0) {

                    int chr = buf;
                    char tamp[101];

                    if ( Dump )
                        printf("[ RX SERIAL] dec:%3d hex:%02X chr:%c   Instate:%d ,InstateServer:%d\n",
                                buf, buf, (buf >= 32 && buf <= 126) ? buf : '.', instate, instateServer);

                    switch( instate ){
                     case 0 :   // Chr normal
                      if ( chr == FNCT_MNTL ){              // Est-ce une touche de fonction Minitel ?
                        send(server_sock,&buf,1,0);         // oui, envoi le chr au serveur                  

                        instate = 1;
                        break;
                      }
                      if ( chr == 0x1b ){                   // Filtrage ESC PRO1, PRO2, PRO3 
                        instate = 2;
                        break;
                      }
                      if ( chr == 0x1 ){
                        instate = 6;                        // ENQ ROM
                        break;
                      }
                      serial_write_char(fdserial, chr);     // Echo sur le Minitel
                      send(server_sock,&buf,1,0);           // Envoi le chr au serveur                  
                      break;

                     case 1 :                               // Seconde touche de fonction Minitel

                      // Demande de connexion au point d'accès si SOMMAIRE et aucune connexion établie
                      if ( chr == SOMMAIRE && server_sock < 0 ){

                         printf( "[    SYSTEM] Demande de connexion envoyée au point d'accès\n");


                        // --- Socket serveur (Frontal) ---
                        server_sock = socket(AF_INET, SOCK_STREAM, 0);
                        if (server_sock < 0) { 
                            printf("[    SYSTEM] Impossible de se connecter au point d'accès Hydris (%s).\n", hydrisAddr);
                            server_sock = -1; 
                            break;
                        }

                        struct sockaddr_in server_addr;
                        memset(&server_addr, 0, sizeof(server_addr));
                        server_addr.sin_family = AF_INET;
                        server_addr.sin_port = htons(serverPort);
                        inet_pton(AF_INET, hydrisAddr, &server_addr.sin_addr);

                        if (server_sock >= 0 && connect(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                            printf("[    SYSTEM] Impossible de se connecter au point d'accès Hydris (%s).\n", hydrisAddr);
                            close(server_sock);
                            server_sock = -1;
                            break;
                        }
                        if(server_sock >= 0) 
                           fcntl(server_sock, F_SETFL, O_NONBLOCK);

                        // --- Fin socket serveur (Frontal) --

                        sprintf( tampon, "\x1F\x30\x30\x18\x1BHDemande de connexion...\x1BI" );
                        serial_write_string( fdserial, tampon, strlen(tampon) );

                      }else // Fin SOMMAIRE cnx
                        send(server_sock,&buf,1,0);           // Envoi le chr au serveur                  

                      instate=0;
                      break;

                     case 2 :                               // Filtrage SEQ ESC
                     if ( chr == '9' ){                     // PRO1
                         instate = 5;             
                         break;  
                      }     
                      if ( chr == ':' ){                    // PRO2
                         instate = 4;           
                         break;  
                      }                               
                      if (  chr == ';' )                    // PRO3
                        instate = 3;
                      else
                        instate = 0;
                      break;
                      
                     case 3 :                               // 3chr Filtrage SEQ ESC 
                      instate = 4;
                      break;
                      
                     case 4 :                               // 2chr Filtrage SEQ ESC
                      instate = 5;
                      break;

                     case 5 :                               // 1chr Filtrage SEQ ESC
                      instate = 0;
                      break;

                     case 6 :                               // Réponse à ENQ ROM : modèle du Minitel - chr1
                        minitelModel[0]=chr;
                        minitelModel[1]='\0';
                        instate = 7;
                        break;

                     case 7 :                               // ENQ ROM chr2
                        minitelModel[1]=chr;
                        minitelModel[2]='\0';
                        instate = 8;
                        break;

                     case 8 :                               // ENQ ROM chr3   filtrage de la version du firmware
                        instate = 9;
                        break;

                     case 9 :                               // Fin ENQ ROM
                        //printf(" Model : %s", minitelModel);
                        if ( !strcmp( minitelModel, "Cb" ) || !strcmp( minitelModel, "Cc" ) || !strcmp( minitelModel, "Cr" ) || 
                                      !strcmp( minitelModel, "Bc" ) || !strcmp( minitelModel, "Br" ) ){
                            typeMinitel = MINITEL1;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 1\n");
                        }
                        
                        if ( !strcmp( minitelModel, "Bs" ) ){
                            typeMinitel = MINITEL1COLOR;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 1 couleur!\n");
                        }          
                        
                        if ( !strcmp( minitelModel, "Br" ) ){
                            typeMinitel = MINITEL1DIAL;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 1 Dialogue\n");
                        }          

                        if ( !strcmp( minitelModel, "Cu" ) || !strcmp( minitelModel, "B0" ) ){
                            typeMinitel = MINITEL1B;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 1B\n");
                        }

                        if ( !strcmp( minitelModel, "Cv" ) || !strcmp( minitelModel, "Bv" ) ){
                            typeMinitel = MINITEL2;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 2\n");
                        }                        
                        
                        if ( !strcmp( minitelModel, "Ay" ) ){
                            typeMinitel = MINITEL5;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 5\n");
                        }          

                        if ( !strcmp( minitelModel, "Cd" ) || !strcmp( minitelModel, "Cf" ) || !strcmp( minitelModel, "Cw" ) ){
                            typeMinitel = MINITEL10;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 10\n");
                        }

                        if ( !strcmp( minitelModel, "Cw" ) ){
                            typeMinitel = MINITEL10B;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 10B\n");
                        }          

                        if ( !strcmp( minitelModel, "Cz" ) || !strcmp( minitelModel, "Bz" ) ){
                            typeMinitel = MINITEL12;
                            printf( "[    SYSTEM] Vous utilisez un Minitel 12\n");
                        }                 

                        if ( !strcmp( minitelModel, "Cp" ) ){
                            typeMinitel = MINITELMAGIS;
                            printf( "[    SYSTEM] Vous utilisez un Minitel Magis\n");
                        }              
                        
                        if ( typeMinitel == MINITEL_UNKNOWN )
                           printf( "[    SYSTEM] Tiens, vous utilisez un Minitel que je ne connais pas (%s)\n", minitelModel);

                        instate = 0;
                        break;

                    } // Fin switch        

                } else if(n == 0) {
                    printf("[   SERIAL] Connexion fermée\n");
                    close(fdserial);
                    fdserial = -1;
                    return 0;
                }
            } // Fin lecture série 

                    
            //
            //       --- Lecture depuis le frontal ---
            //

            tNow = time(NULL);
            double elapsed = difftime(tNow, tDeb);
            if (elapsed > 1){

                if (server_sock >= 0 && FD_ISSET(server_sock, &read_fds)) {
                    unsigned char buf;
                    int cv;
                    ssize_t n = read(server_sock, &buf, 1);
                    if (n > 0) {

                        // Traitement des données du frontal Hydris
                        int chr = buf;
                        char tamp[101];

                        if ( Dump )
                            printf("[RX<FRONTAL] dec:%3d hex:%02X chr:%c ,Instate:%d\n",
                                    buf, buf, (buf >= 32 && buf <= 126) ? buf : '.', instateServer);

                        switch( instateServer ){
                        case 0 :   // Recherche ESC
                            serial_write_char(fdserial, chr);    // Echo sur Minitel

                            if ( chr == '\x1B' )
                            instateServer = 1;
                            break;

                        case 1 : 
                            if ( chr == '\x3A' ){ 
                            serial_write_char(fdserial, chr); // Echo sur Minitel.  TEST POUR MIN MAJ 
                            instateServer = 2;
                            break;
                            }
                            if ( chr == '%' ){                   // % = 2eme chr de ACK demandé par Hydris
                            instateServer = 20;
                            break;
                            }
                            serial_write_char(fdserial, chr);    // Echo sur Minitel
                            instateServer = 0;
                            break;

                        case 2 : 
                            if ( chr == 16 )                     // 0x10
                            instateServer = 3;
                            else{
                            serial_write_char(fdserial, chr); // Echo sur Minitel.  TEST POUR MIN MAJ 
                            instateServer = 0;
                            }
                            break;

                        case 3 :                                // Demande de changement de débit par Hydris (pour garantir une expérience 80')
                            switch( chr ){
                            case 0x41 : // 1200

                                if ( !speedMode ) break;

                                tx_buffer_clear(&COM_TXBuffer);
                                serial_write_string( fdserial, "\x1B:kd", 4 );  // Configuration du Minitel en 1200bds
                                tx_buffer_flush(fdserial, &COM_TXBuffer, currentSpeed );
                                delay_ms(200);
                                serial_set_baudrate(fdserial, B1200 );          // Changement de la vitesse du port local
                                currentSpeed = 1200;

                                printf("[   SERVEUR] Demande de changement de débit : 1200 bauds \n");
                                break;  

                            case 0x42 : // 4800
                                if ( !speedMode ) break;

                                tx_buffer_clear(&COM_TXBuffer);
                                delay_ms(200);
                                serial_write_string( fdserial, "\x1b:kv", 4 );  // Configuration du Minitel en 4800bds
                                tx_buffer_flush(fdserial, &COM_TXBuffer, currentSpeed );
                                delay_ms(200);
                                serial_set_baudrate(fdserial, B4800 );          // Changement de la vitesse du port local
                                currentSpeed = 4800;

                                printf("[   SERVEUR] Demande de changement de débit : 4800 bauds \n");
                                break;

                            case 0x43 : // 9600
                                if ( !speedMode ) break;

                                if ( typeMinitel == MINITEL1 || typeMinitel == MINITEL1COLOR || typeMinitel == MINITEL1DIAL )
                                {
                                    printf("[   SERVEUR] Demande de changement de débit à 9600bds, impossible sur un Minitel 1!\n");
                                    break;
                                }
                                tx_buffer_clear(&COM_TXBuffer);
                                serial_write_string( fdserial, "\x1B:k\x7F", 4 );  // Configuration du Minitel en 9600bds
                                tx_buffer_flush(fdserial, &COM_TXBuffer, currentSpeed );
                                delay_ms(200);
                                serial_set_baudrate(fdserial, B9600 );          // Changement de la vitesse du port local
                                currentSpeed = 9600;

                                printf("[   SERVEUR] Demande de changement de débit : 9600 bauds \n");
                                break;    
                            } // Fin switch
                            instateServer = 0;
                            break;

                            case 20 : // ACK 
                                if ( chr == 'L' )
                                instateServer = 21;
                                else 
                                instateServer = 0;
                                break;

                            case 21 : // ACK 
                                if ( chr == 'e' )
                                instateServer = 22;
                                else 
                                instateServer = 0;
                                break;

                            case 22 : // ACK 
                                if ( chr == 'N' )
                                instateServer = 23;
                                else 
                                instateServer = 0;
                                break;

                            case 23 : // Fin de ACK

                                if ( chr == 'A' ){  // Demande de ACK complète 

                                ip_compress(externalIP, bufIP);  // IP Externe sur 4 octets pour inclure dans la trame de ACK
                                unsigned char ackmsg[]={'\x1B','\x39','\x68',65+typeMinitel,bufIP[0],bufIP[1],bufIP[2],bufIP[3],'\x0D'};

                                for(int i=0; i < 9; i++){
                                    send(server_sock,&ackmsg[i],1,0);  
                                    delay_ms(50);  
                                }
                                if ( Dump )
                                    printf("[TX>FRONTAL] ACK OK\n");
                                printf( "[    SYSTEM] Connexion acceptée par le point d'accès.\n");

                                }
                                instateServer = 0;
                            break;

                        } // Fin switch

                    } else if(n == 0) {
                        printf("[   FRONTAL] Déconnexion par le serveur distant.\n");
                        printf("[   FRONTAL] Attente d'une demande de connexion.\n");
                        close(server_sock);
                        server_sock = -1;
          
                        // Affiche les derniers messages 
                        tx_buffer_flush(fdserial, &COM_TXBuffer, currentSpeed);

                        // Pause pour lecture des derniers message avant de repasser en attente
                        delay_ms(3000); 

                        // Réaffiche page d'attente de démarrage
                        sprintf( tampon, "\x1b\x3a\x32\x7E\x14\x1F\x30\x30\x18\x0C\x1b\x42________________________________________\x1BQ \x18\x0A\x0D\x1BU \x18\x1BM"
                                        "\x1B@ Passerelle de connexion vers Hydris!\x1BM\x1F\x44\x41\x1B\x42~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\x1FHL\x1B]"
                                        "\x1B\x42 Sommaire \x1B\\ Connexion\x1FJF\x1B]\x1B\x45 Shift + Cnx/Fin \x1B\\\x1b\x42 D\031Beconnexion\x0a\x0a\x0a\x0d"
                                        "\x1F\x56\x46\x1b\x42 2026, Minitel is still alive!\x1B@\x1B;\x60XR\x1B:iE" );
                        serial_write_string( fdserial, tampon, strlen(tampon) );

                        //break;
                    }
                } // Fin lecture frontal 
            }

            // --- Lecture clavier ---
            if(FD_ISSET(STDIN_FILENO,&read_fds)){
                unsigned char key; if(read(STDIN_FILENO,&key,1)>0){
                    if(key==27){ printf("\n [Echap] détecté. Sortie.\n\n"); break; }
                    if(key=='d'||key=='D'){ Dump=!Dump; printf("             → Dump=%s\n",Dump?"ON":"OFF"); }
                    
                    if(key=='p'||key=='P'){ printf("             Pause 5 sec\n"); delay_ms(5000); }
                    
                    // MIN = D(own) & MAJ = U(p)
                    if(key=='a'){ printf("MIN\n"); serial_write_string( fdserial, "\x1B:iE", 4 ); }     // MIN
                    if(key=='b'){ printf("MAJ\n"); serial_write_string( fdserial, "\x1B:jE", 4 ); }     // MAJ

                    // Divers tests                    
                    if(key=='1'){ printf("1\n"); serial_write_string( fdserial, "\x1F\x30\x30\x1BHbient\031Cot deconnect\031Be \x1BI\x0A", 31 ); }
                    if(key=='2'){ printf("2\n"); serial_write_string( fdserial, "\x1B:jE", 4 ); }
                    if(key=='3'){ printf("3\n"); serial_write_string( fdserial, "\x1B\x39\x67", 3 ); }
                    if(key=='4'){ printf("4\n"); serial_write_string( fdserial, "\x1B\x3B\x6A\x58\x41", 5 ); }

                    if (key==10)
                        printf("\n");

                }

            } // Lecture clavier, si quelque chose en attente

        }   // Fin activity

    } // Fin while

    // Sortie après ECHAP 
    if(server_sock >= 0) close(server_sock);
    
    if(fdserial >= 0){

        if ( speedMode ){ 
          tx_buffer_flush(fdserial, &COM_TXBuffer, currentSpeed);
          serial_write_string( fdserial, "\x1B:kd", 4 );  // Configuration du Minitel en 1200bds
          tx_buffer_flush(fdserial, &COM_TXBuffer, 4800 );
          delay_ms(200);
          serial_set_baudrate(fdserial, B1200 );   
       }
       strcpy( tampon, "\x0C\x1F\x30\x30\x18See you soon!\x0a"); 
       //strcpy( tampon, "\x1F\x30\x30\x18\x46in de communication.\x0a"); 
       serial_write_string( fdserial, tampon, strlen(tampon) );
       tx_buffer_flush(fdserial, &COM_TXBuffer, 1200 );
       delay_ms(400); 
       close(fdserial);
    }
    free( tampon );
    return 0;
}
