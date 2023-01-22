#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
void* monitorizacion() {
	// Abrir el dispositivo de la USART
	int fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);
	if (fd < 0) {
		perror("Error al abrir el dispositivo de la USART");
		return 1;
	}
	FILE *logger;

	// Configurar la USART
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag = CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	tcsetattr(fd, TCSANOW, &options);
	char *buf;
	// Leer datos de la USART y mostrarlos en la pantalla
	while (1) {
		int n = read(fd, &buf, sizeof(*buf));
		if (n < 0) {
			perror("Error al leer de la USART");
			break;
		}
		logger = fopen("logger.txt", "a");
		if (logger == NULL) {
			printf("ERROR AL ABRIR EL LOGGER");
			exit(1);
		}
		//write(STDOUT_FILENO, &buf, n);
		fprintf(logger, &buf);
		fclose(logger);
	}

	// Cerrar el dispositivo de la USART
	close(fd);
}

void* user_interface() {

	printf("Se escribirán los valores de los sensores en el fichero logger.txt. \n"
			"Introduzca el número 1 para terminar la escritura en el fichero: ");
	while (1) {
		int valorUsuario;
		scanf("%d", &valorUsuario);
		if (valorUsuario==1)
		{
			exit(1);
		}
	}
}
int main() {
	pthread_t tp[2];
	if (pthread_create(&tp[0], NULL, (void*) monitorizacion, NULL) != 0) {
		fprintf(stderr, "Error al crear thread para la monitorizacion\n");
		exit(1);
	};
	if (pthread_create(&tp[1], NULL, (void*) user_interface, NULL) != 0) {
		fprintf(stderr, "Error al crear thread para la user_interface\n");
		exit(1);
	};
	pthread_join(tp[0], NULL);
	pthread_join(tp[1], NULL);
	while (1) {
	}
	return 0;
}
