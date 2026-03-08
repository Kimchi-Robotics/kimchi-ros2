#!/usr/bin/env python3
"""
clean_map.py - Limpieza de mapas de ocupación ROS 2 (PGM/PNG)

Elimina puntos aislados (e.g. piernas del operador durante teleop)
preservando paredes finas y estructura del mapa.

Uso:
    python3 clean_map.py map.png
    python3 clean_map.py map.png --min-area 50 --output map_clean.png
    python3 clean_map.py map.png --preview  # Muestra antes/después sin guardar
"""

import argparse
import cv2
import numpy as np
from pathlib import Path


def load_map(path: str) -> np.ndarray:
    """Carga el mapa en escala de grises."""
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"No se pudo cargar: {path}")
    return img


def binarize_obstacles(
    gray: np.ndarray,
    free_thresh: int = 230,
    occupied_thresh: int = 50,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Separa el mapa en máscara de obstáculos y máscara de zonas desconocidas.

    En mapas ROS estándar:
      - 254 (blanco) = libre
      - 0   (negro)  = ocupado
      - 205 (gris)   = desconocido

    Returns:
        obstacles: máscara binaria donde 255 = obstáculo
        unknown:   máscara binaria donde 255 = desconocido
    """
    obstacles = (gray < occupied_thresh).astype(np.uint8) * 255
    unknown = ((gray > occupied_thresh) & (gray < free_thresh)).astype(np.uint8) * 255
    return obstacles, unknown


def remove_small_blobs(
    mask: np.ndarray,
    min_area: int = 30,
) -> np.ndarray:
    """
    Elimina blobs (contornos) con área menor a min_area píxeles.
    Esto es lo que saca tus piernas sin afectar paredes finas.
    """
    cleaned = mask.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    removed = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            cv2.drawContours(cleaned, [cnt], -1, 0, thickness=cv2.FILLED)
            removed += 1

    print(f"  Contornos encontrados: {len(contours)}")
    print(f"  Blobs eliminados (área < {min_area}px): {removed}")
    return cleaned


def morphological_cleanup(
    mask: np.ndarray,
    kernel_size: int = 3,
    iterations: int = 1,
) -> np.ndarray:
    """
    Opening morfológico suave para limpiar ruido residual.
    Se aplica DESPUÉS del filtrado por área para refinar bordes.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
    return opened


def reconstruct_map(
    original: np.ndarray,
    clean_obstacles: np.ndarray,
    unknown: np.ndarray,
    free_value: int = 254,
    occupied_value: int = 0,
    unknown_value: int = 205,
) -> np.ndarray:
    """
    Reconstruye el mapa con los valores estándar de ROS.
    """
    result = np.full_like(original, free_value)
    result[unknown > 0] = unknown_value
    result[clean_obstacles > 0] = occupied_value
    return result


def clean_map(
    input_path: str,
    min_area: int = 30,
    kernel_size: int = 3,
    morph_iterations: int = 1,
    free_thresh: int = 230,
    occupied_thresh: int = 50,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Pipeline completo de limpieza.

    Returns:
        (original, cleaned) para comparación
    """
    print(f"Cargando mapa: {input_path}")
    original = load_map(input_path)
    print(f"  Tamaño: {original.shape[1]}x{original.shape[0]} px")

    print("Binarizando obstáculos...")
    obstacles, unknown = binarize_obstacles(original, free_thresh, occupied_thresh)

    print("Eliminando blobs pequeños (filtrado por área)...")
    filtered = remove_small_blobs(obstacles, min_area)

    if morph_iterations > 0:
        print("Aplicando opening morfológico...")
        filtered = morphological_cleanup(filtered, kernel_size, morph_iterations)

    print("Reconstruyendo mapa...")
    cleaned = reconstruct_map(original, filtered, unknown)

    return original, cleaned


def create_comparison(original: np.ndarray, cleaned: np.ndarray) -> np.ndarray:
    """Crea imagen lado a lado + diff para visualización."""
    # Diff: rojo = lo que se eliminó
    diff_color = cv2.cvtColor(original, cv2.COLOR_GRAY2BGR)
    removed = (original < 50) & (cleaned >= 200)  # era obstáculo, ahora es libre
    diff_color[removed] = [0, 0, 255]  # Rojo = eliminado

    orig_color = cv2.cvtColor(original, cv2.COLOR_GRAY2BGR)
    clean_color = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)

    # Labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    h = original.shape[0]
    cv2.putText(orig_color, "Original", (10, 30), font, 0.8, (0, 0, 255), 2)
    cv2.putText(clean_color, "Limpio", (10, 30), font, 0.8, (0, 180, 0), 2)
    cv2.putText(diff_color, "Eliminado (rojo)", (10, 30), font, 0.8, (0, 0, 255), 2)

    comparison = np.hstack([orig_color, clean_color, diff_color])
    return comparison

def clean_and_save_map(
    input_path: str,
    min_area: int = 30,
    kernel_size: int = 3,
    morph_iterations: int = 1,
    free_thresh: int = 230,
    occupied_thresh: int = 50,
    output_path: str = None,
):
    original, cleaned = clean_map(
        input_path,
        min_area=min_area,
        kernel_size=kernel_size,
        morph_iterations=morph_iterations,
        free_thresh=free_thresh,
        occupied_thresh=occupied_thresh,
    )

    # Output path
    input_path = Path(input_path)
    if output_path:
        output_path = output_path
    else:
        output_path = str(input_path.parent / f"{input_path.stem}_clean{input_path.suffix}")

    cv2.imwrite(output_path, cleaned)
    print(f"\nMapa limpio guardado en: {output_path}")

# Example usage that worked with kimchi: python3 map_post_processing.py kimchi_map.png -o map_cleaned.png --min-area 1 --morph-iterations 0 --preview
# Main queda comentado para testing.
# def main():
#     parser = argparse.ArgumentParser(
#         description="Limpia mapas de ocupación ROS eliminando puntos aislados"
#     )
#     parser.add_argument("input", help="Ruta al mapa (PNG/PGM)")
#     parser.add_argument(
#         "-o", "--output",
#         help="Ruta de salida (default: <input>_clean.<ext>)",
#     )
#     parser.add_argument(
#         "--min-area",
#         type=int,
#         default=30,
#         help="Área mínima en píxeles para conservar un obstáculo (default: 30)",
#     )
#     parser.add_argument(
#         "--kernel-size",
#         type=int,
#         default=3,
#         help="Tamaño del kernel morfológico (default: 3)",
#     )
#     parser.add_argument(
#         "--morph-iterations",
#         type=int,
#         default=1,
#         help="Iteraciones de opening morfológico, 0 para desactivar (default: 1)",
#     )
#     parser.add_argument(
#         "--free-thresh",
#         type=int,
#         default=230,
#         help="Threshold para espacio libre (default: 230)",
#     )
#     parser.add_argument(
#         "--occupied-thresh",
#         type=int,
#         default=50,
#         help="Threshold para ocupado (default: 50)",
#     )
#     parser.add_argument(
#         "--preview",
#         action="store_true",
#         help="Genera imagen de comparación antes/después",
#     )
#     parser.add_argument(
#         "--no-morph",
#         action="store_true",
#         help="Solo filtrado por área, sin opening morfológico",
#     )

#     args = parser.parse_args()

#     morph_iter = 0 if args.no_morph else args.morph_iterations

#     original, cleaned = clean_map(
#         args.input,
#         min_area=args.min_area,
#         kernel_size=args.kernel_size,
#         morph_iterations=morph_iter,
#         free_thresh=args.free_thresh,
#         occupied_thresh=args.occupied_thresh,
#     )

#     # Output path
#     input_path = Path(args.input)
#     if args.output:
#         output_path = args.output
#     else:
#         output_path = str(input_path.parent / f"{input_path.stem}_clean{input_path.suffix}")

#     cv2.imwrite(output_path, cleaned)
#     print(f"\nMapa limpio guardado en: {output_path}")

#     if args.preview:
#         comp = create_comparison(original, cleaned)
#         comp_path = str(input_path.parent / f"{input_path.stem}_comparison.png")
#         cv2.imwrite(comp_path, comp)
#         print(f"Comparación guardada en: {comp_path}")

#     print("¡Listo!")


# if __name__ == "__main__":
#     main()