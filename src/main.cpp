#include <Arduino.h>
#include <BTS7960.h>

// ============================================================================
// NO DE ATUACAO (ESP32 padrao): injecao de condicoes de operacao por comando serial
//
// Comandos (115200 baud):
//   h = saudavel                  g = falha de engrenagem (emulada)
//   s = subtensao (PWM 75%)       0 = parar
//   z = zera o contador do encoder    p = imprime o contador (calibracao)
// ============================================================================

// ---- Motor sob teste: apenas ele gira; o outro fica parado -----------------
enum Alvo { ALVO_JGB37, ALVO_N20 };
const Alvo ALVO = ALVO_JGB37 ;
const bool USA_ENCODER = (ALVO == ALVO_JGB37);   // o N20 nao tem encoder

// ---- Pinos -----------------------------------------------------------------
// TB6612FNG + Motor CC N20 (5V)
#define TB_PWMA 25
#define TB_AIN1 26
#define TB_AIN2 27
#define TB_STBY 14
#define TB_PWM_CHANNEL 0
#define TB_PWM_FREQ 1000
#define TB_PWM_RESOLUTION 8

// BTS7960 + Motor CC JGB37-520 (12V)
#define BTS_LEN 22
#define BTS_REN 23
#define BTS_LPWM 33
#define BTS_RPWM 32

// Encoder do JGB37-520 (alimentar o encoder com 3,3 V; so o canal A e usado)
#define ENC_A 18

// ---- Parametros das condicoes ----------------------------------------------
const uint8_t PWM_NOMINAL   = 255;   // saudavel, e tambem a base da falha (igual nas duas classes)
const uint8_t PWM_SUBTENSAO = 191;   // 75%: ~9 V no JGB37, ~3,75 V no N20

// Falha de engrenagem emulada: uma queda de PWM por volta do eixo de saida
const uint8_t PWM_FALHA_QUEDA = 160;       // 63%. Evite valores muito baixos: o motor pode parar na queda
const float   FRACAO_QUEDA = 0.07f;        // com encoder: a queda dura ~7% da volta
const unsigned long DURACAO_QUEDA_MS = 40; // sem encoder: duracao da queda
const float F_ROT_HZ_JGB37 = 1.83f;        // MEDIR: rotacao real do eixo de saida a 100% (veja [ROT])
const float F_ROT_HZ_N20   = 1.0f;         // nominal (60 RPM); medir por video ou sensor

// Pulsos do encoder por volta do EIXO DE SAIDA. 0 = nao calibrado (usa o periodo em tempo).
// Calibrar: modo parado ('0'), 'z', gire o eixo de saida 10 voltas a mao, 'p', divida por 10.
const uint32_t PULSOS_POR_VOLTA = 0;

// ---- Estado ----------------------------------------------------------------
BTS7960 motorBTS(BTS_LEN, BTS_REN, BTS_LPWM, BTS_RPWM);

enum Modo { PARADO, SAUDAVEL, FALHA_ENGRENAGEM, SUBTENSAO };
Modo modo = PARADO;

int ultimoDuty = -1;

volatile uint32_t pulsos = 0;
bool encoderVivo = true;
unsigned long tRelatorio = 0;
uint32_t p0Relatorio = 0;

void IRAM_ATTR contaPulso() { pulsos++; }

bool bobinaLivre = false;

void religarDriver() {
  if (bobinaLivre) {
    digitalWrite(BTS_LEN, HIGH);
    digitalWrite(BTS_REN, HIGH);
    bobinaLivre = false;
    ultimoDuty = -1;
  }
}

uint32_t lerPulsos() {
  noInterrupts();
  const uint32_t p = pulsos;
  interrupts();
  return p;
}

const char* nomeModo(Modo m) {
  switch (m) {
    case SAUDAVEL:         return "SAUDAVEL";
    case FALHA_ENGRENAGEM: return "FALHA_ENGRENAGEM";
    case SUBTENSAO:        return "SUBTENSAO";
    default:               return "PARADO";
  }
}

// Aplica o duty (0 a 255) ao motor sob teste; o outro fica parado
void aplicarPwm(uint8_t duty) {
  if (duty == ultimoDuty) return;      // so escreve quando muda
  ultimoDuty = duty;

  if (ALVO == ALVO_JGB37) {
    motorBTS.pwm = duty;
    motorBTS.front();
    ledcWrite(TB_PWM_CHANNEL, 0);
  } else {
    ledcWrite(TB_PWM_CHANNEL, duty);
    motorBTS.pwm = 0;
    motorBTS.front();
  }
}

// Falha de engrenagem: uma queda de PWM por volta do eixo de saida
uint8_t dutyFalhaEngrenagem() {
  if (USA_ENCODER && PULSOS_POR_VOLTA > 0 && encoderVivo) {
    // Presa ao angulo: a queda ocorre sempre na mesma posicao do eixo
    const uint32_t posicao = lerPulsos() % PULSOS_POR_VOLTA;
    const uint32_t largura = (uint32_t)(FRACAO_QUEDA * PULSOS_POR_VOLTA);
    return posicao < largura ? PWM_FALHA_QUEDA : PWM_NOMINAL;
  }

  // Sem encoder (ou nao calibrado): periodo em tempo, igual a rotacao do eixo
  const float f = (ALVO == ALVO_JGB37) ? F_ROT_HZ_JGB37 : F_ROT_HZ_N20;
  const unsigned long periodo = (unsigned long)(1000.0f / f);
  return (millis() % periodo) < DURACAO_QUEDA_MS ? PWM_FALHA_QUEDA : PWM_NOMINAL;
}

void tratarComando(char c) {
  const char k = tolower(c);
  if (k == 'h' || k == 'g' || k == 's' || k == '0') religarDriver();
  switch (tolower(c)) {
    case 'h':
      modo = SAUDAVEL;
      Serial.println("[MODO] SAUDAVEL");
      break;
    case 'g':
      modo = FALHA_ENGRENAGEM;
      Serial.println("[MODO] FALHA DE ENGRENAGEM (emulada)");
      if (USA_ENCODER && PULSOS_POR_VOLTA == 0) {
        Serial.println("[AVISO] PULSOS_POR_VOLTA nao calibrado: usando o periodo em tempo (F_ROT_HZ)");
      }
      break;
    case 's':
      modo = SUBTENSAO;
      Serial.println("[MODO] SUBTENSAO (PWM 75%)");
      break;
    case '0':
      modo = PARADO;
      Serial.println("[MODO] PARADO");
      break;
    case 'z':
      noInterrupts();
      pulsos = 0;
      interrupts();
      p0Relatorio = 0;
      Serial.println("[ENC] contador zerado");
      break;
    case 'p':
      Serial.printf("[ENC] pulsos=%lu\n", (unsigned long)lerPulsos());
      break;
        case 'f':   // roda livre: driver desabilitado, para girar o eixo a mao
      modo = PARADO;
      aplicarPwm(0);
      digitalWrite(BTS_LEN, LOW);
      digitalWrite(BTS_REN, LOW);
      bobinaLivre = true;
      Serial.println("[MODO] RODA LIVRE (driver desabilitado)");
      break;
    default:
      break;
  }
}

// Uma linha por segundo: modo e rotacao (quando ha encoder)
void relatorioPeriodico() {
  const unsigned long agora = millis();
  if (agora - tRelatorio < 1000) return;
  const float dt = (agora - tRelatorio) / 1000.0f;
  tRelatorio = agora;

  if (!USA_ENCODER) {
    Serial.printf("[STATUS] modo=%s\n", nomeModo(modo));
    return;
  }

  const uint32_t p = lerPulsos();
  const uint32_t delta = p - p0Relatorio;
  p0Relatorio = p;

  if (modo != PARADO) {
    encoderVivo = (delta > 0);
    if (!encoderVivo) {
      Serial.println("[AVISO] encoder sem pulsos: conferir a fiacao (usando o periodo em tempo)");
    }
  }

  if (PULSOS_POR_VOLTA > 0) {
    const float hz = delta / (float)PULSOS_POR_VOLTA / dt;
    Serial.printf("[ROT] modo=%s rot=%.2f Hz (%.1f RPM)\n", nomeModo(modo), hz, hz * 60.0f);
  } else {
    Serial.printf("[ROT] modo=%s pulsos/s=%.0f (calibrar PULSOS_POR_VOLTA)\n", nomeModo(modo), delta / dt);
  }
}

void setup() {
  Serial.begin(115200);

  // TB6612 (N20)
  pinMode(TB_AIN1, OUTPUT);
  pinMode(TB_AIN2, OUTPUT);
  pinMode(TB_STBY, OUTPUT);
  ledcSetup(TB_PWM_CHANNEL, TB_PWM_FREQ, TB_PWM_RESOLUTION);
  ledcAttachPin(TB_PWMA, TB_PWM_CHANNEL);
  digitalWrite(TB_STBY, HIGH);
  digitalWrite(TB_AIN1, HIGH);
  digitalWrite(TB_AIN2, LOW);
  ledcWrite(TB_PWM_CHANNEL, 0);

  // BTS7960 (JGB37-520)
  motorBTS.begin();
  motorBTS.enable();
  motorBTS.pwm = 0;
  motorBTS.front();

  // Encoder (so no JGB37)
  if (USA_ENCODER) {
    pinMode(ENC_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), contaPulso, RISING);
  }

  tRelatorio = millis();
  Serial.printf("[PRONTO] alvo=%s | h=saudavel g=falha_engrenagem s=subtensao 0=parar z/p=calibrar encoder\n",
                ALVO == ALVO_JGB37 ? "JGB37-520" : "N20");
}

void loop() {
  while (Serial.available()) tratarComando(Serial.read());

  switch (modo) {
    case PARADO:           aplicarPwm(0); break;
    case SAUDAVEL:         aplicarPwm(PWM_NOMINAL); break;
    case SUBTENSAO:        aplicarPwm(PWM_SUBTENSAO); break;
    case FALHA_ENGRENAGEM: aplicarPwm(dutyFalhaEngrenagem()); break;
  }

  relatorioPeriodico();
}