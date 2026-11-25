/*
  Tela de dicas do modulo Labirinto (Grupo 6)

  Plataforma: ESP32 TTGO T-Display (TFT 240x135).
  Dependencias: TFT_eSPI.

  Funcionamento:
  - Roda nas placas dos outros grupos enquanto o G6 e o modulo ativo.
  - A central repassa cada posicao de bomba (ex.: "B4") no topico jogo/modulo/<ID_DA_PLACA>/dica/6.
  - dicaG6_iniciar(tft): chamar quando jogo/modulo/ativo virar "6"; limpa as dicas e mostra a espera.
  - dicaG6_receber(tft, payload): chamar ao receber a dica; guarda e exibe ate 3 posicoes.
  - Nao altera a rotacao da tela (funciona com rotacao 1 ou 3).
*/

#pragma once

// bibliotecas do display e de manipulacao de texto
#include <TFT_eSPI.h>
#include <string.h>
#include <ctype.h>

// lista das posicoes recebidas (ate 3, com ate 7 caracteres cada)
static const int DICA_G6_MAX = 3;
static char dicaG6_lista[DICA_G6_MAX][8];
static int  dicaG6_qtd = 0;

// cor de fundo da tela de dicas
static const uint16_t DICA_G6_FUNDO = TFT_BLACK;

// limpa a tela e desenha o titulo com a linha separadora
static void dicaG6_cabecalho(TFT_eSPI& tft) {
  tft.fillScreen(DICA_G6_FUNDO);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, DICA_G6_FUNDO);
  tft.drawString("LABIRINTO (G6)", 120, 12);
  tft.drawFastHLine(10, 26, 220, TFT_DARKGREY);
}

// redesenha a tela conforme a quantidade de dicas recebidas
static void dicaG6_desenhar(TFT_eSPI& tft) {
  dicaG6_cabecalho(tft);

  // sem dicas: mostra a espera e a instrucao para o jogador
  if (dicaG6_qtd == 0) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, DICA_G6_FUNDO);
    tft.drawString("Aguardando dica...", 120, 52);

    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY, DICA_G6_FUNDO);
    tft.drawString("Se aparecer a posicao de uma bomba,", 120, 84);
    tft.drawString("avise quem esta jogando o LABIRINTO", 120, 98);
    tft.drawString("para a bola NAO passar por ela.", 120, 112);
    return;
  }

  // titulo no singular ou plural conforme a quantidade
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, DICA_G6_FUNDO);
  tft.drawString(dicaG6_qtd == 1 ? "BOMBA EM:" : "BOMBAS EM:", 120, 40);

  // monta uma linha com as posicoes separadas por dois espacos, sem estourar o buffer
  char linha[32] = "";
  for (int i = 0; i < dicaG6_qtd; i++) {
    if (i > 0) strncat(linha, "  ", sizeof(linha) - strlen(linha) - 1);
    strncat(linha, dicaG6_lista[i], sizeof(linha) - strlen(linha) - 1);
  }
  // reduz o tamanho da fonte conforme aumenta a quantidade, para caber na largura da tela
  tft.setTextSize(dicaG6_qtd == 1 ? 5 : (dicaG6_qtd == 2 ? 4 : 3));
  tft.setTextColor(TFT_RED, DICA_G6_FUNDO);
  tft.drawString(linha, 120, 74);

  // instrucao para repassar a dica a quem joga o labirinto
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, DICA_G6_FUNDO);
  tft.drawString("Avise quem esta jogando o LABIRINTO:", 120, 108);
  tft.drawString("a bola NAO pode passar por ai!", 120, 122);
}

// inicia a tela quando o G6 vira o modulo ativo, descartando dicas antigas
void dicaG6_iniciar(TFT_eSPI& tft) {
  dicaG6_qtd = 0;
  dicaG6_desenhar(tft);
}

// recebe uma dica da central e atualiza a tela
void dicaG6_receber(TFT_eSPI& tft, const char* payload) {
  // normaliza o texto: remove espacos e converte para maiusculo (ex.: " b4\n" -> "B4")
  char pos[8];
  int n = 0;
  for (const char* p = payload; *p && n < (int)sizeof(pos) - 1; p++) {
    if (!isspace((unsigned char)*p)) pos[n++] = toupper((unsigned char)*p);
  }
  pos[n] = '\0';
  if (n == 0) return;

  // ignora posicao ja recebida
  for (int i = 0; i < dicaG6_qtd; i++)
    if (strcmp(dicaG6_lista[i], pos) == 0) return;

  // guarda a nova posicao se ainda houver espaco na lista
  if (dicaG6_qtd < DICA_G6_MAX) {
    strcpy(dicaG6_lista[dicaG6_qtd++], pos);
  }
  dicaG6_desenhar(tft);
}