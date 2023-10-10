#define amostras 2000
#define sensor 13
#define E 3400 // Módulo de elasticidade do material medido em MPa
#define L 300 // Comprimento do braço em mm
#define b 94 // Largura da barra em mm
#define h 2 // Espessura da barra em mm

float zero = 0, strain = 0, def = 0, delta_def;
float calib = 0, w, m, p, e, sigma;

void setup() {
  Serial.begin(115200);
  delay(5000);
  for (int i = 0; i < amostras; i++) {
    zero += analogRead(sensor);
    delay(10);
  }
  Serial.println("Posicione a carga de valor conhecido");
  delay(5000);
  Serial.println("Início da calibração");
  for (int k = 0; k < amostras; k++){
    calib += analogRead(sensor);
    delay(10);
  }
  zero = zero / amostras;
  calib = calib / amostras;
  Serial.print("Valor de calibração: "); Serial.println(calib);
  w = (b*(pow(h, 2)))/6.0;
}

void loop() {
  for (int j = 0; j < 11; j++) {
    def = analogRead(sensor);
    strain += def;
    delay(10);
  }
  strain = strain/11;
  p = 0.3885*strain/calib;
  m = p * L;
  sigma = m/w;
  e = (sigma/E)*1000;  
  Serial.println(e, 10);
}
