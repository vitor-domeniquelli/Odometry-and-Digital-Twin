#!/usr/bin/env python3
"""
plot_metrics.py

Script standalone para análise de métricas e geração de gráficos do Gêmeo Digital.
Requisito para o artigo do IEEE Latin America Transactions.

NOTA IMPORTANTE: 
Para que este script funcione, o odom_node e o serial_bridge_node (ou um nó de 
gravação dedicado) devem possuir um mecanismo de logging que salve os dados em arquivos CSV 
com as colunas esperadas (timestamp, x, y, theta) e (timestamp, latency_ms).

Uso:
  python3 plot_metrics.py --odom odom_log.csv --gt gt_log.csv [--latency latency_log.csv]
"""

import argparse
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

def load_csv(filepath):
    """Lê um arquivo CSV ignorando cabeçalhos e retorna um array numpy."""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                # Tenta converter a linha inteira para float
                data.append([float(val) for val in row])
            except ValueError:
                # Ignora a linha se houver texto (ex: cabeçalho)
                pass
    return np.array(data)

def normalize_angle(angle):
    """Normaliza um ângulo para o intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def main():
    parser = argparse.ArgumentParser(description="Gera gráficos e métricas de odometria vs ground truth.")
    parser.add_argument('--odom', required=True, help="Caminho para o CSV da odometria (t, x, y, theta)")
    parser.add_argument('--gt', required=True, help="Caminho para o CSV do ground truth (t, x, y, theta)")
    parser.add_argument('--latency', required=False, help="Caminho para o CSV de latência (t, latency_ms)")
    args = parser.parse_args()

    # 1. Carregamento dos dados
    print("Carregando dados...")
    odom_data = load_csv(args.odom)
    gt_data = load_csv(args.gt)

    t_odom = odom_data[:, 0]
    x_odom = odom_data[:, 1]
    y_odom = odom_data[:, 2]
    theta_odom = odom_data[:, 3]

    t_gt = gt_data[:, 0]
    x_gt_raw = gt_data[:, 1]
    y_gt_raw = gt_data[:, 2]
    theta_gt_raw = gt_data[:, 3]

    # 2. Interpolação: Alinhar Ground Truth aos timestamps da Odometria
    # Como as frequências são diferentes (Odometria ~20Hz, GT ~30Hz), interpolamos o GT
    x_gt = np.interp(t_odom, t_gt, x_gt_raw)
    y_gt = np.interp(t_odom, t_gt, y_gt_raw)
    
    # Obs: Interpolação de ângulos crua pode falhar perto de pi/-pi, mas para trajetórias
    # sem múltiplas voltas completas (como Linha e Quadrado) np.interp funciona perfeitamente.
    theta_gt = np.interp(t_odom, t_gt, theta_gt_raw)

    # 3. Cálculo das Métricas de Erro de Pose (APE)
    ape_series = np.sqrt((x_odom - x_gt)**2 + (y_odom - y_gt)**2)
    
    mean_ape = np.mean(ape_series)
    max_ape = np.max(ape_series)
    rms_ape = np.sqrt(np.mean(ape_series**2))
    final_pos_error = ape_series[-1]
    
    # Erro de orientação final (graus)
    diff_theta = normalize_angle(theta_odom[-1] - theta_gt[-1])
    final_ori_error_deg = math.degrees(abs(diff_theta))

    # Distância Total (Odometria)
    diff_x = np.diff(x_odom)
    diff_y = np.diff(y_odom)
    dist_total = np.sum(np.sqrt(diff_x**2 + diff_y**2))

    # Drift por metro
    drift_per_meter = final_pos_error / dist_total if dist_total > 0 else 0.0

    # 4. Exibição da Tabela de Resultados no Terminal
    print("\n" + "="*50)
    print(" MÉTRICAS DE ODOMETRIA VS GROUND TRUTH")
    print("="*50)
    print(f" Distância Total Percorrida: {dist_total:.4f} m")
    print(f" APE Médio:                  {mean_ape:.4f} m")
    print(f" APE Máximo:                 {max_ape:.4f} m")
    print(f" APE RMS:                    {rms_ape:.4f} m")
    print(f" Erro Final de Posição:      {final_pos_error:.4f} m")
    print(f" Erro Final de Orientação:   {final_ori_error_deg:.2f} graus")
    print(f" Drift por Metro:            {drift_per_meter:.4f} m/m")
    print("="*50)

    # 5. Análise de Latência (Se fornecida)
    if args.latency:
        lat_data = load_csv(args.latency)
        t_lat = lat_data[:, 0]
        lat_ms = lat_data[:, 1]

        mean_lat = np.mean(lat_ms)
        max_lat = np.max(lat_ms)
        std_lat = np.std(lat_ms)
        
        # Frequência efetiva = 1 / tempo médio entre pacotes
        dt_lat = np.diff(t_lat)
        dt_mean = np.mean(dt_lat)
        freq_hz = 1.0 / dt_mean if dt_mean > 0 else 0.0

        print("\n" + "="*50)
        print(" MÉTRICAS DO GÊMEO DIGITAL (LATÊNCIA)")
        print("="*50)
        print(f" Frequência Efetiva (ROS):   {freq_hz:.2f} Hz")
        print(f" Latência Média:             {mean_lat:.2f} ms")
        print(f" Latência Máxima:            {max_lat:.2f} ms")
        print(f" Desvio Padrão Latência:     {std_lat:.2f} ms")
        print("="*50)

    # ==========================================
    # 6. GERAÇÃO DE GRÁFICOS
    # ==========================================
    print("\nGerando gráficos...")

    # Figura 1: Trajetórias
    plt.figure(figsize=(8, 8))
    plt.plot(x_odom, y_odom, label='Odometria Estimada', color='blue', linewidth=2)
    plt.plot(x_gt, y_gt, label='Ground Truth (ArUco)', color='green', linewidth=2, linestyle='--')
    
    # Marcadores de Início (Estrela) e Fim (Círculo)
    plt.scatter(x_odom[0], y_odom[0], color='blue', marker='*', s=150, label='Início Odom')
    plt.scatter(x_odom[-1], y_odom[-1], color='blue', marker='o', s=80, label='Fim Odom')
    plt.scatter(x_gt[0], y_gt[0], color='green', marker='*', s=150, label='Início GT')
    plt.scatter(x_gt[-1], y_gt[-1], color='green', marker='o', s=80, label='Fim GT')

    plt.title("Comparação de Trajetórias: Odometria vs Ground Truth")
    plt.xlabel("Eixo X (m)")
    plt.ylabel("Eixo Y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # Aspect ratio igual para não distorcer o quadrado
    plt.savefig('trajetorias.png', dpi=300, bbox_inches='tight')
    
    # Figura 2: APE ao longo do tempo
    plt.figure(figsize=(10, 4))
    plt.plot(t_odom - t_odom[0], ape_series, color='red', linewidth=1.5)
    plt.axhline(mean_ape, color='black', linestyle='--', label=f'Média ({mean_ape:.3f} m)')
    
    plt.title("Erro Absoluto de Pose (APE) ao Longo do Tempo")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Erro de Posição (m)")
    plt.legend()
    plt.grid(True)
    plt.savefig('ape_tempo.png', dpi=300, bbox_inches='tight')

    # Figura 3: Histograma de Latência (Condicional)
    if args.latency:
        plt.figure(figsize=(8, 5))
        plt.hist(lat_ms, bins=30, color='purple', alpha=0.7, edgecolor='black')
        plt.axvline(mean_lat, color='red', linestyle='dashed', linewidth=2, label=f'Média ({mean_lat:.1f} ms)')
        
        plt.title("Distribuição da Latência de Comunicação (ESP32 -> ROS2)")
        plt.xlabel("Latência (ms)")
        plt.ylabel("Frequência")
        plt.legend()
        plt.grid(axis='y', alpha=0.75)
        plt.savefig('latencia.png', dpi=300, bbox_inches='tight')

    print("Imagens salvas com sucesso: trajetorias.png, ape_tempo.png" + (", latencia.png" if args.latency else ""))
    
    # Exibe os gráficos gerados
    plt.show()

if __name__ == '__main__':
    main()