import cv2
import numpy as np

# ==========================================
# CONFIGURAÇÕES DO PADRÃO XADREZ
# ==========================================
CHESSBOARD_SIZE = (10, 7)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

def calcular_rms_por_imagem(objpoints, imgpoints, camera_matrix, dist_coeffs, rvecs, tvecs):
    """
    Calcula o erro de reprojeção RMS global e por imagem.
    Valores de referência:
      < 0.5 px  → excelente
      0.5–1.0   → bom (aceitável para artigo)
      1.0–2.0   → razoável
      > 2.0     → ruim, recalibrar
    """
    erros = []
    for i in range(len(objpoints)):
        imgpoints_proj, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        erro = cv2.norm(imgpoints[i], imgpoints_proj, cv2.NORM_L2) / len(imgpoints_proj)
        erros.append(erro)

    rms_global = np.sqrt(np.mean(np.array(erros) ** 2))
    return rms_global, erros

def avaliar_calibracao(rms):
    if rms < 0.5:
        return "✅ EXCELENTE — ótimo para o artigo"
    elif rms < 1.0:
        return "✅ BOM — aceitável para o artigo"
    elif rms < 2.0:
        return "⚠️  RAZOÁVEL — pode melhorar com mais poses variadas"
    else:
        return "❌ RUIM — recalibre com mais variação de ângulo e posição"

def main():
    cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("======================================================")
    print("CALIBRAÇÃO DA CÂMERA")
    print("Dicas para boa calibração:")
    print("  - Incline o padrão em 3D (não só gire no plano)")
    print("  - Cubra os 4 cantos da imagem")
    print("  - Varie a distância (perto e longe)")
    print("  - Mire para pelo menos 20 poses")
    print("")
    print("Pressione 'c' para CAPTURAR uma pose")
    print("Pressione 'q' para CALCULAR e SALVAR")
    print("======================================================")

    captures = 0
    gray = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao acessar a câmera.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_chess, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        display_frame = frame.copy()

        if ret_chess:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners_refined, ret_chess)
            status_color = (0, 255, 0)
            status_text = "Padrao detectado — pressione 'c'"
        else:
            status_color = (0, 0, 255)
            status_text = "Padrao NAO detectado"

        cv2.putText(display_frame, f"Capturas: {captures}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        cv2.putText(display_frame, status_text, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        cv2.imshow('Calibracao', display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c') and ret_chess:
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            captures += 1
            print(f"[{captures}] Pose capturada com sucesso!")

        elif key == ord('q'):
            break

    # ==========================================
    # CÁLCULO DA CALIBRAÇÃO
    # ==========================================
    if captures > 0 and gray is not None:
        print(f"\nCalculando calibração com {captures} poses...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        if ret:
            # Erro RMS global e por imagem
            rms_global, erros_por_imagem = calcular_rms_por_imagem(
                objpoints, imgpoints, camera_matrix, dist_coeffs, rvecs, tvecs
            )

            print("\n" + "="*54)
            print("RESULTADO DA CALIBRAÇÃO")
            print("="*54)
            print(f"\nErro de Reprojeção RMS: {rms_global:.4f} px")
            print(f"Avaliação: {avaliar_calibracao(rms_global)}")

            print(f"\nErro por imagem (px):")
            for i, erro in enumerate(erros_por_imagem):
                barra = "█" * int(erro * 20)
                flag = " ⚠️" if erro > 1.0 else ""
                print(f"  Pose {i+1:02d}: {erro:.4f}  {barra}{flag}")

            # Identifica poses problemáticas
            pior_idx = int(np.argmax(erros_por_imagem))
            melhor_idx = int(np.argmin(erros_por_imagem))
            print(f"\n  Melhor pose: #{melhor_idx+1} ({erros_por_imagem[melhor_idx]:.4f} px)")
            print(f"  Pior pose:   #{pior_idx+1} ({erros_por_imagem[pior_idx]:.4f} px)")

            print(f"\nMatriz da Câmera (K):")
            print(f"  fx={camera_matrix[0,0]:.1f}  fy={camera_matrix[1,1]:.1f}")
            print(f"  cx={camera_matrix[0,2]:.1f}  cy={camera_matrix[1,2]:.1f}")

            print(f"\nCoeficientes de Distorção (D):")
            labels = ['k1', 'k2', 'p1', 'p2', 'k3']
            for label, val in zip(labels, dist_coeffs[0]):
                print(f"  {label} = {val:.6f}")

            # Aviso se k2 ou k3 muito altos
            k2 = abs(dist_coeffs[0][1])
            k3 = abs(dist_coeffs[0][4])
            if k2 > 1.0 or k3 > 3.0:
                print("\n⚠️  ATENÇÃO: k2 ou k3 altos indicam calibração instável.")
                print("   Tente recalibrar com poses mais próximas da câmera")
                print("   e com maior variação de inclinação 3D.")

            # Salva os arquivos
            np.save('camera_matrix.npy', camera_matrix)
            np.save('dist_coeffs.npy', dist_coeffs)
            print("\n✅ Arquivos 'camera_matrix.npy' e 'dist_coeffs.npy' salvos!")

        else:
            print("\n❌ Falha ao calibrar. Tente mais poses com ângulos diferentes.")
    else:
        print("\nNenhuma imagem capturada. Calibração cancelada.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()