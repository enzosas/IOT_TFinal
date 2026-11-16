#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>

// =====================================================
// HARDWARE E CONFIGURAÇÕES
// =====================================================

TFT_eSPI tela = TFT_eSPI();
Adafruit_MPU6050 mpu;

const int W = 240;
const int H = 135;
const int LED_PIN = 4;

const bool DEBUG_BOMBAS = true; // true para desenhar bombas e debugar

// =====================================================
// GRADE: 8 COLUNAS x 7 LINHAS (A-G)
// =====================================================

const int COLUNAS = 8;
const int LINHAS = 7;

const int ESQ = 13;
const int DIR = 239;
const int TOPO = 9;
const int BASE = 134;

const float CEL_W = (float)(DIR - ESQ) / COLUNAS;
const float CEL_H = (float)(BASE - TOPO) / LINHAS;

const int R = 3;

// Objetivo fixo
const int OBJ_COL = COLUNAS - 1; // Coluna 7 (H / 8 visualmente)
const int OBJ_LINHA = LINHAS - 1; // Linha 6 (G visualmente)

// =====================================================
// CORES
// =====================================================

const uint16_t COR_FUNDO  = TFT_BLACK;
const uint16_t COR_GRADE  = 0x1082;
const uint16_t COR_PAREDE = 0xBDF7;
const uint16_t COR_BOLA   = TFT_CYAN;
const uint16_t COR_LABEL  = TFT_RED;

// =====================================================
// FISICA DA BOLA
// =====================================================

float bolaX = 0, bolaY = 0;
float velX = 0, velY = 0;
float refX = 0, refY = 0;
float filtroX = 0, filtroY = 0;

// Controle: a inclinacao define a VELOCIDADE desejada (nao a aceleracao).
// Inclinou -> anda; nivelou -> para. Muito mais facil de dosar.
const float GANHO = 25.0;          // px/s por m/s^2 de inclinacao
const float RESPOSTA = 8.0;        // rapidez com que a bola obedece (menor = mais "escorregadia")
const float ZONA_MORTA = 0.3;
const float VELOCIDADE_MAX = 90.0;
const float SINAL_X = -1.0;
const float SINAL_Y = -1.0;

// Encostar na parede: false = bola para/desliza; true = volta ao inicio (comportamento antigo)
const bool PAREDE_REINICIA = true;

// Assistencia de curva: ao empurrar contra uma parede, a bola e puxada
// para o centro do corredor que tem saida naquela direcao (px/s)
const float ASSISTENCIA = 60.0;

float intencaoX = 0, intencaoY = 0; // inclinacao atual (usada pela assistencia)

unsigned long tempoAnterior = 0;
unsigned long inicioCalibracao = 0;
bool calibrado = false;
int ultimaX = -1, ultimaY = -1;

const float INICIO_X = ESQ + CEL_W * 0.5f;
const float INICIO_Y = TOPO + CEL_H * 0.5f;


// =====================================================
// PAREDES
// =====================================================

struct Parede {
  int x1, y1, x2, y2;
};

// Labirinto perfeito 8x7 tem no maximo 42 paredes internas; 60 da folga.
const int MAX_PAREDES = 60;

Parede paredes[MAX_PAREDES];
int numParedes = 0;

// hOcupada[y][x]: parede entre a linha y-1 e y, na coluna x
// vOcupada[y][x]: parede entre a coluna x-1 e x, na linha y
bool hOcupada[LINHAS + 1][COLUNAS] = {};
bool vOcupada[LINHAS][COLUNAS + 1] = {};

// =====================================================
// BOMBAS INVISIVEIS
// =====================================================

struct Bomba {
  int coluna, linha;
};

const int NUM_BOMBAS = 3;
Bomba bombas[NUM_BOMBAS];
bool matrizBombas[LINHAS][COLUNAS] = {false};

float px(int x) { return ESQ + x * CEL_W; }
float py(int y) { return TOPO + y * CEL_H; }

void atualizarMatrizBombas() {
  memset(matrizBombas, 0, sizeof(matrizBombas));
  for (int i = 0; i < NUM_BOMBAS; i++) {
    if (bombas[i].coluna != -1) {
      matrizBombas[bombas[i].linha][bombas[i].coluna] = true;
    }
  }
}

bool celulaTemBomba(int c, int l) {
  return matrizBombas[l][c];
}

// =====================================================
// UTILITARIOS DA GRADE
// =====================================================

const int DX[4] = {0, 0, -1, 1};
const int DY[4] = {-1, 1, 0, 0};
const int NUM_CELULAS = LINHAS * COLUNAS;

// Existe passagem (sem parede) entre (cx,cy) e a vizinha (nx,ny)?
bool passagemLivre(int cx, int cy, int nx, int ny) {
  if (nx > cx) return !vOcupada[cy][nx];
  if (nx < cx) return !vOcupada[cy][cx];
  if (ny > cy) return !hOcupada[ny][cx];
  return !hOcupada[cy][cx];
}

// Coloca/remove a parede entre duas celulas vizinhas
void definirParede(int ax, int ay, int bx, int by, bool valor) {
  if (ax != bx) vOcupada[ay][max(ax, bx)] = valor;
  else          hOcupada[max(ay, by)][ax] = valor;
}

// =====================================================
// BFS GENERICO
// =====================================================
// Busca de (oX,oY) ate (dX,dY).
// evitarBombas: nao entra em celulas com bomba.
// (eAx,eAy)-(eBx,eBy): passagem proibida (use -1 para nenhuma).
// Se outX != nullptr, devolve o caminho na ordem origem -> destino.

bool buscarCaminho(int oX, int oY, int dX, int dY, bool evitarBombas,
                   int eAx, int eAy, int eBx, int eBy,
                   int outX[], int outY[], int *outLen) {
  bool visitado[LINHAS][COLUNAS] = {};
  int8_t paiX[LINHAS][COLUNAS], paiY[LINHAS][COLUNAS];
  int8_t filaX[NUM_CELULAS], filaY[NUM_CELULAS];
  int ini = 0, fim = 0;

  filaX[fim] = oX; filaY[fim] = oY; fim++;
  visitado[oY][oX] = true;
  bool achou = false;

  while (ini < fim) {
    int cx = filaX[ini], cy = filaY[ini]; ini++;
    if (cx == dX && cy == dY) { achou = true; break; }

    for (int i = 0; i < 4; i++) {
      int nx = cx + DX[i], ny = cy + DY[i];
      if (nx < 0 || nx >= COLUNAS || ny < 0 || ny >= LINHAS) continue;
      if (visitado[ny][nx]) continue;
      if (!passagemLivre(cx, cy, nx, ny)) continue;
      if ((cx == eAx && cy == eAy && nx == eBx && ny == eBy) ||
          (cx == eBx && cy == eBy && nx == eAx && ny == eAy)) continue;
      if (evitarBombas && matrizBombas[ny][nx]) continue;

      visitado[ny][nx] = true;
      paiX[ny][nx] = cx; paiY[ny][nx] = cy;
      filaX[fim] = nx; filaY[fim] = ny; fim++;
    }
  }

  if (achou && outX != nullptr) {
    int len = 0, cx = dX, cy = dY;
    while (true) {
      outX[len] = cx; outY[len] = cy; len++;
      if (cx == oX && cy == oY) break;
      int pxa = paiX[cy][cx];
      cy = paiY[cy][cx]; cx = pxa;
    }
    for (int i = 0; i < len / 2; i++) {
      int tx = outX[i]; outX[i] = outX[len - 1 - i]; outX[len - 1 - i] = tx;
      int ty = outY[i]; outY[i] = outY[len - 1 - i]; outY[len - 1 - i] = ty;
    }
    *outLen = len;
  }
  return achou;
}

// =====================================================
// PASSO 1: LABIRINTO PERFEITO (DFS ITERATIVO)
// =====================================================
// Comeca com todas as paredes internas e cava uma arvore geradora:
// todas as celulas acessiveis e exatamente 1 caminho entre quaisquer duas.

void gerarArvore() {
  memset(hOcupada, 0, sizeof(hOcupada));
  memset(vOcupada, 0, sizeof(vOcupada));
  for (int y = 1; y < LINHAS; y++)
    for (int x = 0; x < COLUNAS; x++) hOcupada[y][x] = true;
  for (int y = 0; y < LINHAS; y++)
    for (int x = 1; x < COLUNAS; x++) vOcupada[y][x] = true;

  bool visitado[LINHAS][COLUNAS] = {};
  int8_t pilhaX[NUM_CELULAS], pilhaY[NUM_CELULAS];
  int sp = 0;

  visitado[0][0] = true;
  pilhaX[sp] = 0; pilhaY[sp] = 0; sp++;

  // Mantem a regra do codigo antigo: sem parede logo abaixo do ponto de partida
  definirParede(0, 0, 0, 1, false);
  visitado[1][0] = true;
  pilhaX[sp] = 0; pilhaY[sp] = 1; sp++;

  while (sp > 0) {
    int cx = pilhaX[sp - 1], cy = pilhaY[sp - 1];
    int opcoes[4], n = 0;
    for (int i = 0; i < 4; i++) {
      int nx = cx + DX[i], ny = cy + DY[i];
      if (nx >= 0 && nx < COLUNAS && ny >= 0 && ny < LINHAS && !visitado[ny][nx]) opcoes[n++] = i;
    }
    if (n == 0) { sp--; continue; }

    int d = opcoes[random(0, n)];
    int nx = cx + DX[d], ny = cy + DY[d];
    definirParede(cx, cy, nx, ny, false);
    visitado[ny][nx] = true;
    pilhaX[sp] = nx; pilhaY[sp] = ny; sp++;
  }
}

// =====================================================
// PASSO 2: DESVIOS + UMA BOMBA EM CADA DESVIO
// =====================================================
// Para cada bomba, escolhe uma parede cuja remocao cria um caminho
// alternativo ate o objetivo (um "atalho" que contorna parte da rota segura).
// A bomba vai num trecho desse desvio que NAO pertence a rota segura.
// Os trechos fora da rota de desvios diferentes nao se sobrepoem, o que
// garante matematicamente que qualquer caminho diferente da rota segura
// passa por pelo menos uma bomba.

bool criarDesviosComBombas() {
  int rotaX[NUM_CELULAS], rotaY[NUM_CELULAS], rotaLen = 0;
  buscarCaminho(0, 0, OBJ_COL, OBJ_LINHA, false, -1, -1, -1, -1, rotaX, rotaY, &rotaLen);

  bool naRota[LINHAS][COLUNAS] = {};
  for (int i = 0; i < rotaLen; i++) naRota[rotaY[i]][rotaX[i]] = true;

  // Lista todas as paredes internas como pares de celulas
  int8_t cAx[100], cAy[100], cBx[100], cBy[100];
  int nc = 0;
  for (int y = 1; y < LINHAS; y++)
    for (int x = 0; x < COLUNAS; x++)
      if (hOcupada[y][x]) { cAx[nc] = x; cAy[nc] = y - 1; cBx[nc] = x; cBy[nc] = y; nc++; }
  for (int y = 0; y < LINHAS; y++)
    for (int x = 1; x < COLUNAS; x++)
      if (vOcupada[y][x]) { cAx[nc] = x - 1; cAy[nc] = y; cBx[nc] = x; cBy[nc] = y; nc++; }

  // Embaralha
  for (int i = 0; i < nc; i++) {
    int r = random(i, nc);
    int8_t t;
    t = cAx[i]; cAx[i] = cAx[r]; cAx[r] = t;
    t = cAy[i]; cAy[i] = cAy[r]; cAy[r] = t;
    t = cBx[i]; cBx[i] = cBx[r]; cBx[r] = t;
    t = cBy[i]; cBy[i] = cBy[r]; cBy[r] = t;
  }

  bool usada[LINHAS][COLUNAS] = {}; // celulas fora da rota ja usadas por outro desvio
  int escolhidas[NUM_BOMBAS];
  int nb = 0;

  for (int k = 0; k < nc && nb < NUM_BOMBAS; k++) {
    // Caminho na arvore entre as duas celulas da parede = o ciclo que sera criado
    int cicX[NUM_CELULAS], cicY[NUM_CELULAS], cicLen = 0;
    buscarCaminho(cAx[k], cAy[k], cBx[k], cBy[k], false, -1, -1, -1, -1, cicX, cicY, &cicLen);

    int celulasNaRota = 0, fora[NUM_CELULAS], nf = 0;
    bool conflito = false;
    for (int i = 0; i < cicLen; i++) {
      if (naRota[cicY[i]][cicX[i]]) celulasNaRota++;
      else {
        if (usada[cicY[i]][cicX[i]]) { conflito = true; break; }
        fora[nf++] = i;
      }
    }
    // Precisa compartilhar ao menos um trecho com a rota (senao nao e alternativa
    // ao objetivo) e ter ao menos uma celula fora dela (onde a bomba vai).
    if (conflito || celulasNaRota < 2 || nf == 0) continue;

    for (int i = 0; i < nf; i++) usada[cicY[fora[i]]][cicX[fora[i]]] = true;
    int e = fora[random(0, nf)];
    bombas[nb] = {cicX[e], cicY[e]};
    escolhidas[nb] = k;
    nb++;
  }

  if (nb < NUM_BOMBAS) return false;

  for (int i = 0; i < NUM_BOMBAS; i++) {
    int k = escolhidas[i];
    definirParede(cAx[k], cAy[k], cBx[k], cBy[k], false);
  }
  atualizarMatrizBombas();
  return true;
}

// =====================================================
// VALIDACAO (1 E APENAS 1 CAMINHO SEGURO)
// =====================================================
// Pela construcao isso ja e garantido; a checagem e barata (~20 BFS em
// 56 celulas) e fica como rede de seguranca.

bool validarMapaRigoroso() {
  int rotaX[NUM_CELULAS], rotaY[NUM_CELULAS], rotaLen = 0;
  if (!buscarCaminho(0, 0, OBJ_COL, OBJ_LINHA, true, -1, -1, -1, -1, rotaX, rotaY, &rotaLen)) return false;

  // Se proibir qualquer passo da rota ainda deixa chegar sem bomba,
  // existe um segundo caminho seguro.
  for (int i = 0; i < rotaLen - 1; i++) {
    if (buscarCaminho(0, 0, OBJ_COL, OBJ_LINHA, true,
                      rotaX[i], rotaY[i], rotaX[i + 1], rotaY[i + 1],
                      nullptr, nullptr, nullptr)) return false;
  }
  return true;
}

// Converte as grades h/v em segmentos para desenho e colisao,
// juntando paredes vizinhas alinhadas num segmento so.
void montarListaParedes() {
  numParedes = 0;
  for (int y = 1; y < LINHAS; y++) {
    int x = 0;
    while (x < COLUNAS) {
      if (!hOcupada[y][x]) { x++; continue; }
      int ini = x;
      while (x < COLUNAS && hOcupada[y][x]) x++;
      if (numParedes < MAX_PAREDES) paredes[numParedes++] = {ini, y, x, y};
    }
  }
  for (int x = 1; x < COLUNAS; x++) {
    int y = 0;
    while (y < LINHAS) {
      if (!vOcupada[y][x]) { y++; continue; }
      int ini = y;
      while (y < LINHAS && vOcupada[y][x]) y++;
      if (numParedes < MAX_PAREDES) paredes[numParedes++] = {x, ini, x, y};
    }
  }
}

void gerarMapaValido() {
  unsigned long t0 = micros();
  int tentativas = 0;

  while (true) {
    tentativas++;
    gerarArvore();
    if (criarDesviosComBombas() && validarMapaRigoroso()) break;
    if (tentativas % 20 == 0) yield();
  }

  montarListaParedes();
  Serial.printf("Mapa gerado em %lu us (%d tentativa(s))\n", micros() - t0, tentativas);
}

// =====================================================
// COMPONENTES VISUAIS E FISICA
// =====================================================

void desenharObjetivo() {
  int x = roundf(px(OBJ_COL));
  int y = roundf(py(OBJ_LINHA));
  tela.fillRect(x + 1, y + 1, CEL_W - 1, CEL_H - 1, TFT_GREEN);
}

void desenharBombasVisualmente() {
  if (!DEBUG_BOMBAS) return;
  for (int i = 0; i < NUM_BOMBAS; i++) {
    int cx = roundf(px(bombas[i].coluna) + CEL_W * 0.5f);
    int cy = roundf(py(bombas[i].linha) + CEL_H * 0.5f);
    tela.fillCircle(cx, cy, R + 2, TFT_RED);
  }
}

void desenharGrade() {
  for (int x = 1; x < COLUNAS; x++) tela.drawLine(roundf(px(x)), TOPO, roundf(px(x)), BASE, COR_GRADE);
  for (int y = 1; y < LINHAS; y++) tela.drawLine(ESQ, roundf(py(y)), DIR, roundf(py(y)), COR_GRADE);
  tela.drawRect(ESQ, TOPO, DIR - ESQ + 1, BASE - TOPO + 1, COR_GRADE);
}

void desenharLabels() {
  tela.setTextColor(COR_LABEL, COR_FUNDO);
  tela.setTextDatum(MC_DATUM);
  tela.setTextSize(1);
  for (int c = 0; c < COLUNAS; c++) {
    char texto[3]; snprintf(texto, sizeof(texto), "%d", c + 1);
    tela.drawString(texto, roundf(ESQ + (c + 0.5f) * CEL_W), 4);
  }
  for (int l = 0; l < LINHAS; l++) {
    char texto[2] = {(char)('A' + l), '\0'};
    tela.drawString(texto, 5, roundf(TOPO + (l + 0.5f) * CEL_H));
  }
}

void desenharParedes() {
  for (int i = 0; i < numParedes; i++) {
    Parede& p = paredes[i];
    int x1 = roundf(px(p.x1)), y1 = roundf(py(p.y1));
    int x2 = roundf(px(p.x2)), y2 = roundf(py(p.y2));
    tela.drawLine(x1, y1, x2, y2, COR_PAREDE);
    if (p.x1 == p.x2) tela.drawLine(x1 + 1, y1, x2 + 1, y2, COR_PAREDE);
    else tela.drawLine(x1, y1 + 1, x2, y2 + 1, COR_PAREDE);
  }
}

void desenharEstaticos() {
  desenharGrade();
  desenharObjetivo();
  desenharBombasVisualmente();
  desenharParedes();
}

void desenharMapa() {
  tela.fillScreen(COR_FUNDO);
  desenharEstaticos();
  desenharLabels();
}

void desenharBola() {
  int x = roundf(bolaX), y = roundf(bolaY);
  tela.fillCircle(x, y, R, COR_BOLA);
  tela.drawCircle(x, y, R, TFT_WHITE);
  ultimaX = x; ultimaY = y;
}

// Apaga so a bola antiga e redesenha linhas/paredes por cima, sem fillScreen.
// Redesenhar a tela inteira a cada pixel derrubava o FPS e deixava o
// controle atrasado.
void atualizarBola() {
  if (ultimaX >= 0) tela.fillCircle(ultimaX, ultimaY, R, COR_FUNDO);
  desenharEstaticos();
  desenharBola();
}

void reiniciarBola() {
  bolaX = INICIO_X; bolaY = INICIO_Y;
  velX = 0; velY = 0;
  desenharMapa();
  desenharBola();
}

void animarExplosao() {
  unsigned long inicio = millis();
  while (millis() - inicio < 500) {
    tela.fillScreen(TFT_RED); delay(100);
    tela.fillScreen(TFT_BLACK); delay(100);
  }
}

void animarVitoria() {
  Serial.println("OBJETIVO ALCANCADO!");
  unsigned long inicio = millis();
  while (millis() - inicio < 500) {
    tela.fillScreen(TFT_GREEN); delay(100);
    tela.fillScreen(TFT_BLACK); delay(100);
  }
}

bool colidiuComParede(float x, float y, const Parede& p) {
  float x1 = px(p.x1), y1 = py(p.y1), x2 = px(p.x2), y2 = py(p.y2);
  float dx = x2 - x1, dy = y2 - y1;
  float comprimento2 = dx * dx + dy * dy;
  if (comprimento2 <= 0.0f) return false;
  float t = constrain(((x - x1) * dx + (y - y1) * dy) / comprimento2, 0.0f, 1.0f);
  float distX = x - (x1 + t * dx), distY = y - (y1 + t * dy);
  float raio = R + 1.0f;
  return (distX * distX + distY * distY <= raio * raio);
}

bool existeColisaoParede(float x, float y) {
  if (x - R < ESQ || x + R > DIR || y - R < TOPO || y + R > BASE) return true;
  for (int i = 0; i < numParedes; i++) if (colidiuComParede(x, y, paredes[i])) return true;
  return false;
}

bool verificarVitoria() {
  return ((int)((bolaX - ESQ) / CEL_W) == OBJ_COL && (int)((bolaY - TOPO) / CEL_H) == OBJ_LINHA);
}

bool verificarBomba() {
  int coluna = (int)((bolaX - ESQ) / CEL_W);
  int linha = (int)((bolaY - TOPO) / CEL_H);
  if (coluna < 0 || coluna >= COLUNAS || linha < 0 || linha >= LINHAS) return false;
  return celulaTemBomba(coluna, linha);
}

// Existe saida da celula (c,l) para a vizinha (c+dc, l+dl)?
bool temSaida(int c, int l, int dc, int dl) {
  int nc = c + dc, nl = l + dl;
  if (c < 0 || c >= COLUNAS || l < 0 || l >= LINHAS) return false;
  if (nc < 0 || nc >= COLUNAS || nl < 0 || nl >= LINHAS) return false;
  return passagemLivre(c, l, nc, nl);
}

// Assistencia de curva: a bola foi barrada num eixo enquanto o jogador
// empurrava nessa direcao. Puxa a bola, no outro eixo, para o centro do
// corredor (atual ou vizinho) que tem saida para onde o jogador quer ir.
void assistirCurva(bool barradaEmX, int dir, float maximo) {
  int c = constrain((int)((bolaX - ESQ) / CEL_W), 0, COLUNAS - 1);
  int l = constrain((int)((bolaY - TOPO) / CEL_H), 0, LINHAS - 1);

  if (barradaEmX) {
    float centro = TOPO + (l + 0.5f) * CEL_H;
    int viz = (bolaY < centro) ? l - 1 : l + 1;
    int alvo = l;
    if (!temSaida(c, l, dir, 0) && temSaida(c, viz, dir, 0) && temSaida(c, l, 0, viz - l)) alvo = viz;
    float p = constrain(TOPO + (alvo + 0.5f) * CEL_H - bolaY, -maximo, maximo);
    if (!existeColisaoParede(bolaX, bolaY + p)) bolaY += p;
  } else {
    float centro = ESQ + (c + 0.5f) * CEL_W;
    int viz = (bolaX < centro) ? c - 1 : c + 1;
    int alvo = c;
    if (!temSaida(c, l, 0, dir) && temSaida(viz, l, 0, dir) && temSaida(c, l, viz - c, 0)) alvo = viz;
    float p = constrain(ESQ + (alvo + 0.5f) * CEL_W - bolaX, -maximo, maximo);
    if (!existeColisaoParede(bolaX + p, bolaY)) bolaX += p;
  }
}

bool moverBola(float deslocX, float deslocY, float dt) {
  float distancia = sqrtf(deslocX * deslocX + deslocY * deslocY);
  int passos = max(1, (int)ceilf(distancia / 1.5f));
  float passoX = deslocX / passos;
  float passoY = deslocY / passos;
  float assist = ASSISTENCIA * dt / passos;

  for (int i = 0; i < passos; i++) {
    float novoX = bolaX + passoX;
    if (!existeColisaoParede(novoX, bolaY)) bolaX = novoX;
    else if (PAREDE_REINICIA) { reiniciarBola(); return true; }
    else {
      // Para no eixo X e, se o jogador quer ir para os lados, ajuda a alinhar
      if (fabs(intencaoX) > fabs(intencaoY) && intencaoX != 0)
        assistirCurva(true, passoX > 0 ? 1 : -1, assist);
      velX = 0; passoX = 0;
    }

    float novoY = bolaY + passoY;
    if (!existeColisaoParede(bolaX, novoY)) bolaY = novoY;
    else if (PAREDE_REINICIA) { reiniciarBola(); return true; }
    else {
      if (fabs(intencaoY) > fabs(intencaoX) && intencaoY != 0)
        assistirCurva(false, passoY > 0 ? 1 : -1, assist);
      velY = 0; passoY = 0;
    }

    if (verificarVitoria()) {
      velX = 0; velY = 0;
      animarVitoria(); reiniciarBola();
      return true;
    }

    if (verificarBomba()) {
      Serial.println("BOMBA ATIVADA!");
      velX = 0; velY = 0;
      animarExplosao(); reiniciarBola();
      return true;
    }
  }
  return false;
}

// =====================================================
// SETUP E TELA DE CARREGAMENTO
// =====================================================

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  tela.init(); 
  tela.setRotation(3); 
  
  // Tela de Carregamento
  tela.fillScreen(COR_FUNDO);
  tela.setTextColor(TFT_WHITE, COR_FUNDO);
  tela.setTextDatum(MC_DATUM);
  tela.setTextSize(2);
  tela.drawString("Gerando", W / 2, (H / 2) - 15);
  tela.drawString("Labirinto...", W / 2, (H / 2) + 10);
  
  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("MPU6050 nao encontrado!");
    while (true) { digitalWrite(LED_PIN, LOW); delay(500); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  randomSeed((uint32_t)micros() ^ (uint32_t)analogRead(34));
  bolaX = INICIO_X; bolaY = INICIO_Y;

  gerarMapaValido();
  
  desenharMapa(); 
  desenharBola();

  inicioCalibracao = millis(); tempoAnterior = millis();
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long agora = millis();
  float dt = (agora - tempoAnterior) / 1000.0f;
  tempoAnterior = agora;

  if (dt > 0.05f) dt = 0.05f;
  if (dt <= 0.0f) dt = 0.001f;

  float leituraX = SINAL_X * a.acceleration.y;
  float leituraY = SINAL_Y * a.acceleration.x;

  filtroX += 0.15f * (leituraX - filtroX);
  filtroY += 0.15f * (leituraY - filtroY);

  if (!calibrado) {
    static float somaX = 0, somaY = 0;
    static int amostras = 0;
    somaX += filtroX; somaY += filtroY; amostras++;

    if (agora - inicioCalibracao >= 2000) {
      refX = somaX / amostras; refY = somaY / amostras;
      filtroX = refX; filtroY = refY;
      calibrado = true; tempoAnterior = agora;
    }
    return;
  }

  float inclinacaoX = filtroX - refX;
  float inclinacaoY = filtroY - refY;

  if (fabs(inclinacaoX) < ZONA_MORTA) inclinacaoX = 0;
  if (fabs(inclinacaoY) < ZONA_MORTA) inclinacaoY = 0;

  intencaoX = inclinacaoX;
  intencaoY = inclinacaoY;

  // Velocidade segue suavemente o alvo definido pela inclinacao
  float alvoX = constrain(inclinacaoX * GANHO, -VELOCIDADE_MAX, VELOCIDADE_MAX);
  float alvoY = constrain(inclinacaoY * GANHO, -VELOCIDADE_MAX, VELOCIDADE_MAX);
  float k = 1.0f - expf(-RESPOSTA * dt);
  velX += (alvoX - velX) * k;
  velY += (alvoY - velY) * k;

  if (alvoX == 0 && fabs(velX) < 1.0f) velX = 0;
  if (alvoY == 0 && fabs(velY) < 1.0f) velY = 0;

  int antigoX = roundf(bolaX), antigoY = roundf(bolaY);

  if (moverBola(velX * dt, velY * dt, dt)) {
    digitalWrite(LED_PIN, HIGH); delay(5); return;
  }

  if (roundf(bolaX) != antigoX || roundf(bolaY) != antigoY) {
    atualizarBola();
  }

  digitalWrite(LED_PIN, HIGH); delay(5);
}