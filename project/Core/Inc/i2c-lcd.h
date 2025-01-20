#include "main.h"

void lcd_init (void);   // Inicializar lcd

void lcd_enviar_cmd (char cmd);  // enviar command al lcd

void lcd_enviar_datos (char datos);  // enviar datos al lcd

void lcd_enviar_string (char *str);  // enviar string al lcd

void lcd_cur(int fila, int col);  // poner el cursor en la posicion especificada fila (0 or 1), col (0-15);

void lcd_clear (void);
