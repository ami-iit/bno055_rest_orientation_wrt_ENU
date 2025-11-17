#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def quat_to_rotation_matrix(q):
    """
    Converte un quaternione (w, x, y, z) in una matrice di rotazione 3x3.
    
    Args:
        q: array di shape (4,) con componenti [w, x, y, z]
    
    Returns:
        Matrice di rotazione 3x3
    """
    w, x, y, z = q
    
    # Normalizza il quaternione
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-12:
        return np.eye(3)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Calcola la matrice di rotazione
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    return R

def rotation_matrix_to_rpy_extrinsic(R):
    """
    Converte una matrice di rotazione in angoli Roll-Pitch-Yaw ESTRINSECI (in radianti).
    Convenzione: ZYX Euler angles con assi fissi globali.
    R = Rz(yaw) * Ry(pitch) * Rx(roll) applicato su assi FISSI
    
    Args:
        R: matrice di rotazione 3x3
    
    Returns:
        Tuple (roll, pitch, yaw) in radianti
    """
    sin_pitch = -R[2, 0]
    
    if np.abs(sin_pitch) >= 1:
        pitch = np.copysign(np.pi / 2, sin_pitch)
        if sin_pitch < 0:
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0
        else:
            yaw = np.arctan2(R[0, 1], R[1, 1])
            roll = 0
    else:
        pitch = np.arcsin(sin_pitch)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    
    return roll, pitch, yaw

def rotation_matrix_to_rpy_intrinsic(R):
    """
    Converte una matrice di rotazione in angoli Roll-Pitch-Yaw INTRINSECI (in radianti).
    Convenzione: rotazioni XYZ con assi che si muovono ad ogni rotazione.
    Prima ruota di roll attorno a X, poi di pitch attorno al nuovo Y, poi di yaw attorno al nuovo Z.
    R = Rx(roll) * Ry'(pitch) * Rz''(yaw) dove gli apici indicano assi ruotati
    
    Matematicamente equivalente a: R = Rz(yaw) * Ry(pitch) * Rx(roll) con ordine invertito!
    
    Args:
        R: matrice di rotazione 3x3
    
    Returns:
        Tuple (roll, pitch, yaw) in radianti per rotazioni intrinsiche
    """
    # Per rotazioni intrinsiche XYZ, la matrice è: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # Ma gli angoli vanno applicati in ordine inverso sugli assi che si modificano
    
    sin_pitch = R[2, 0]
    
    if np.abs(sin_pitch) >= 1:
        pitch = np.copysign(np.pi / 2, sin_pitch)
        if sin_pitch > 0:
            yaw = np.arctan2(R[0, 1], R[1, 1])
            roll = 0
        else:
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0
    else:
        pitch = np.arcsin(sin_pitch)
        roll = np.arctan2(-R[2, 1], R[2, 2])
        yaw = np.arctan2(-R[1, 0], R[0, 0])
    
    return roll, pitch, yaw

def rpy_to_rotation_matrix(roll, pitch, yaw):
    """
    Converte angoli Roll-Pitch-Yaw in matrice di rotazione.
    Ordine: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    
    Args:
        roll, pitch, yaw: angoli in radianti
    
    Returns:
        Matrice di rotazione 3x3
    """
    # Matrice di rotazione attorno a X (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Matrice di rotazione attorno a Y (pitch)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Matrice di rotazione attorno a Z (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Componi nell'ordine: Yaw -> Pitch -> Roll
    R = Rz @ Ry @ Rx
    return R

def compute_mean_rotation(quaternions):
    """
    Calcola la rotazione media da un insieme di quaternioni [w, x, y, z].
    1) Allinea il segno dei quaternioni rispetto al primo (per evitare cancellazioni).
    2) Media i quaternioni normalizzati e rinormalizza il risultato.
    3) Converte il quaternione medio in matrice e la ortonormalizza via SVD (polar decomposition).
    
    Args:
        quaternions: array di shape (N, 4) con quaternioni [w, x, y, z]
    
    Returns:
        Matrice di rotazione media 3x3 (ortonormalizzata)
    """
    qs = np.array(quaternions, dtype=float)
    if qs.size == 0:
        return np.eye(3)

    # Normalizza e allinea il segno rispetto al primo quaternione
    q0 = qs[0]
    n0 = np.linalg.norm(q0)
    if n0 > 0:
        q0 = q0 / n0
    for i in range(len(qs)):
        n = np.linalg.norm(qs[i])
        if n > 0:
            qs[i] = qs[i] / n
        # Evita media tra q e -q (stessa rotazione ma segno opposto)
        if np.dot(qs[i], q0) < 0:
            qs[i] = -qs[i]

    # Media aritmetica e rinormalizzazione
    q_mean = np.mean(qs, axis=0)
    n_mean = np.linalg.norm(q_mean)
    if n_mean < 1e-12:
        R = np.eye(3)
    else:
        q_mean = q_mean / n_mean
        R = quat_to_rotation_matrix(q_mean)

    # Ortonormalizza tramite decomposizione polare (SVD)
    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1
        R_ortho = U @ Vt

    return R_ortho

def load_node_files(inputs_dir):
    """
    Carica tutti i file CSV che iniziano con 'node' dalla cartella inputs.
    Ogni file viene considerato come un nodo separato.
    
    Args:
        inputs_dir: path della cartella con i file di input
    
    Returns:
        Dictionary con {node_name: dataframe}
    """
    files = sorted(glob.glob(os.path.join(inputs_dir, "node*.csv")))
    
    if not files:
        raise FileNotFoundError(f"Nessun file node*.csv trovato in {inputs_dir}")
    
    print(f"\n📂 Trovati {len(files)} file:")
    for f in files:
        print(f"   - {os.path.basename(f)}")
    
    # Ogni file è un nodo separato
    node_data = {}
    for f in files:
        basename = os.path.basename(f)
        # Usa il nome completo del file (senza estensione) come nome del nodo
        node_name = os.path.splitext(basename)[0]
        
        df = pd.read_csv(f)
        df.columns = [c.strip() for c in df.columns]
        
        node_data[node_name] = df
    
    return node_data

def compute_mean_heading_per_node(node_data, n_samples=100):
    """
    Calcola l'heading medio dei primi n_samples per ogni nodo.
    
    Args:
        node_data: dictionary con {node_name: dataframe}
        n_samples: numero di campioni da utilizzare per il calcolo della media
    
    Returns:
        Tuple (mean_rotations, angles_data) dove angles_data contiene gli angoli per ogni nodo
    """
    quat_cols = ["qw", "qx", "qy", "qz"]
    mean_rotations = {}
    angles_data = {}
    
    print(f"\n📊 Calcolo heading medio (primi {n_samples} campioni):")
    
    for node_name, df in node_data.items():
        print(f"\n  🔹 {node_name}:")
        
        # Verifica presenza quaternioni
        if not all(col in df.columns for col in quat_cols):
            print(f"    ✗ Quaternioni non trovati, salto questo nodo")
            continue
        
        # Estrai quaternioni
        quaternions = df[quat_cols].to_numpy()
        
        # Prendi solo i primi n_samples
        n_to_take = min(n_samples, len(quaternions))
        quaternions_subset = quaternions[:n_to_take]
        
        print(f"    ✓ {n_to_take} campioni estratti (su {len(quaternions)} totali)")
        
        # Calcola la rotazione media
        R_mean = compute_mean_rotation(quaternions_subset)
        mean_rotations[node_name] = R_mean
        
        print(f"    ✓ Heading medio calcolato")
        
        # Calcola gli angoli RPY ESTRINSECI (assi fissi)
        roll_ext, pitch_ext, yaw_ext = rotation_matrix_to_rpy_extrinsic(R_mean)
        roll_ext_deg = np.degrees(roll_ext)
        pitch_ext_deg = np.degrees(pitch_ext)
        yaw_ext_deg = np.degrees(yaw_ext)
        
        # Calcola gli angoli RPY INTRINSECI (assi che si muovono)
        roll_int, pitch_int, yaw_int = rotation_matrix_to_rpy_intrinsic(R_mean)
        roll_int_deg = np.degrees(roll_int)
        pitch_int_deg = np.degrees(pitch_int)
        yaw_int_deg = np.degrees(yaw_int)
        
        # Salva gli angoli per la stampa finale
        angles_data[node_name] = {
            'extrinsic': (roll_ext_deg, pitch_ext_deg, yaw_ext_deg),
            'intrinsic': (roll_int_deg, pitch_int_deg, yaw_int_deg)
        }
        
        print(f"\n    📐 Angoli ESTRINSECI (assi fissi globali):")
        print(f"       Roll  (X): {roll_ext_deg:8.3f}°")
        print(f"       Pitch (Y): {pitch_ext_deg:8.3f}°")
        print(f"       Yaw   (Z): {yaw_ext_deg:8.3f}°")
        
        print(f"\n    � Angoli INTRINSECI (assi che si modificano):")
        print(f"       Roll  (X): {roll_int_deg:8.3f}°")
        print(f"       Pitch (Y): {pitch_int_deg:8.3f}°")
        print(f"       Yaw   (Z): {yaw_int_deg:8.3f}°")
        
        # Verifica ricostruzione (solo caso estrinseco)
        R_ext = rpy_to_rotation_matrix(roll_ext, pitch_ext, yaw_ext)
        error_ext = np.linalg.norm(R_mean - R_ext, 'fro')
        
        print(f"\n    🔍 Verifica ricostruzione matrice:")
        print(f"       Errore Frobenius: {error_ext:.6f}")
        if error_ext > 0.01:
            print(f"       ⚠️  Errore significativo nella conversione!")
    
    return mean_rotations, angles_data

def group_nodes_by_prefix(mean_rotations):
    """
    Raggruppa i nodi per prefisso (es: node10_1, node10_2 -> 'node10').
    
    Args:
        mean_rotations: dictionary con {node_name: mean_rotation_matrix}
    
    Returns:
        Dictionary con {prefix: {node_name: mean_rotation_matrix}}
    """
    grouped = {}
    for node_name, R_mean in mean_rotations.items():
        # Estrai il prefisso (es: node10_1 -> node10)
        prefix = node_name.split('_')[0]
        
        if prefix not in grouped:
            grouped[prefix] = {}
        grouped[prefix][node_name] = R_mean
    
    return grouped

def plot_combined_filtered_nodes(mean_rotations, output_dir):
    """
    Crea un grafico unico con tutti i nodi dal 3 al 12 (con o senza suffisso).
    
    Args:
        mean_rotations: dictionary con {node_name: mean_rotation_matrix}
        output_dir: directory dove salvare il grafico
    """
    # Filtra i nodi: node3, node4, ..., node12 (con eventuali suffissi)
    filtered_rotations = {}
    
    for node_name, R_mean in mean_rotations.items():
        # Estrai il numero del nodo dal nome
        # Formato: nodeN o nodeN_M
        if node_name.startswith('node'):
            try:
                # Rimuovi 'node' e prendi la parte numerica prima di eventuale '_'
                parts = node_name[4:].split('_')  # Rimuove 'node' e splitta
                node_num = int(parts[0])
                
                # Considera solo nodi da 3 a 12
                if 3 <= node_num <= 12:
                    filtered_rotations[node_name] = R_mean
            except (ValueError, IndexError):
                continue
    
    if not filtered_rotations:
        print("\n⚠️  Nessun nodo trovato nell'intervallo node3-node12")
        return
    
    print(f"\n🎨 Creazione plot 3D combinato con {len(filtered_rotations)} nodi:")
    for node_name in sorted(filtered_rotations.keys()):
        print(f"   - {node_name}")
    
    # Setup figura
    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')
    
    # Assi del sistema di riferimento
    axis_length = 1.0
    origin = np.array([0, 0, 0])
    
    # Colori per i diversi nodi
    colors_nodes = plt.cm.Set1(np.linspace(0, 1, max(len(filtered_rotations), 3)))
    
    # Disegna il frame di riferimento ENU (East-North-Up) nell'origine
    ref_length = 1.2
    ax.plot([0, ref_length], [0, 0], [0, 0], 'r-', linewidth=4, label='ENU - X (East)', alpha=0.8)
    ax.plot([0, 0], [0, ref_length], [0, 0], 'g-', linewidth=4, label='ENU - Y (North)', alpha=0.8)
    ax.plot([0, 0], [0, 0], [0, ref_length], 'b-', linewidth=4, label='ENU - Z (Up)', alpha=0.8)
    
    # Aggiungi etichette al frame di riferimento
    ax.text(ref_length, 0, 0, 'X (E)', fontsize=11, color='red', weight='bold')
    ax.text(0, ref_length, 0, 'Y (N)', fontsize=11, color='green', weight='bold')
    ax.text(0, 0, ref_length, 'Z (U)', fontsize=11, color='blue', weight='bold')
    
    # Per ogni nodo, plotta gli assi X, Y, Z
    for idx, (node_name, R_mean) in enumerate(sorted(filtered_rotations.items())):
        color = colors_nodes[idx]
        
        # Calcola gli assi ruotati
        x_axis = R_mean @ np.array([axis_length, 0, 0])
        y_axis = R_mean @ np.array([0, axis_length, 0])
        z_axis = R_mean @ np.array([0, 0, axis_length])
        
        # Plotta asse X (in tonalità più scura)
        ax.plot([origin[0], x_axis[0]], 
                [origin[1], x_axis[1]], 
                [origin[2], x_axis[2]], 
                color=color, linewidth=3, linestyle='-', 
                label=f'{node_name} - X', alpha=0.9)
        
        # Plotta asse Y (in tonalità media)
        ax.plot([origin[0], y_axis[0]], 
                [origin[1], y_axis[1]], 
                [origin[2], y_axis[2]], 
                color=color, linewidth=3, linestyle='--', 
                label=f'{node_name} - Y', alpha=0.7)
        
        # Plotta asse Z (in tonalità più chiara)
        ax.plot([origin[0], z_axis[0]], 
                [origin[1], z_axis[1]], 
                [origin[2], z_axis[2]], 
                color=color, linewidth=3, linestyle=':', 
                label=f'{node_name} - Z', alpha=0.6)
        
        # Aggiungi etichette agli estremi degli assi
        ax.text(x_axis[0], x_axis[1], x_axis[2], f'{node_name}-X', 
                fontsize=9, color=color, weight='bold')
        ax.text(y_axis[0], y_axis[1], y_axis[2], f'{node_name}-Y', 
                fontsize=8, color=color)
        ax.text(z_axis[0], z_axis[1], z_axis[2], f'{node_name}-Z', 
                fontsize=8, color=color, style='italic')
    
    # Setup assi
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X (East)', fontsize=12, weight='bold')
    ax.set_ylabel('Y (North)', fontsize=12, weight='bold')
    ax.set_zlabel('Z (Up)', fontsize=12, weight='bold')
    ax.set_title('Heading medio - NODE 3-12 (primi 100 campioni)\nFrame 3D sovrapposti - Sistema ENU (BNO055 standard config)', 
                 fontsize=14, weight='bold')
    
    # Aggiungi griglia
    ax.grid(True, alpha=0.3)
    
    # Legenda (divisa in due colonne per gestire molti elementi)
    ax.legend(loc='upper left', fontsize=8, framealpha=0.9, ncol=2)
    
    # Imposta vista isometrica
    ax.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    
    # Salva il grafico
    save_path = os.path.join(output_dir, "mean_headings_3d_combined_node3_to_12.png")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"   💾 Plot salvato: {save_path}")
    
    # Mostra il plot
    print("\n   ▶️  Apertura finestra plot...")
    plt.show()

def plot_mean_frames_3d_grouped(mean_rotations, output_dir):
    """
    Crea grafici separati per ogni gruppo di nodi (raggruppati per prefisso).
    
    Args:
        mean_rotations: dictionary con {node_name: mean_rotation_matrix}
        output_dir: directory dove salvare i grafici
    """
    # Raggruppa i nodi per prefisso
    grouped_nodes = group_nodes_by_prefix(mean_rotations)
    
    print(f"\n🎨 Creazione plot 3D separati per {len(grouped_nodes)} gruppi:")
    
    for group_name, group_rotations in sorted(grouped_nodes.items()):
        print(f"\n  📊 Gruppo '{group_name}' ({len(group_rotations)} nodi)...")
        
        # Setup figura
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Assi del sistema di riferimento
        axis_length = 1.0
        origin = np.array([0, 0, 0])
        
        # Colori per i diversi nodi del gruppo
        colors_nodes = plt.cm.Set1(np.linspace(0, 1, max(len(group_rotations), 3)))
        
        # Per ogni nodo del gruppo, plotta gli assi X, Y, Z
        for idx, (node_name, R_mean) in enumerate(sorted(group_rotations.items())):
            color = colors_nodes[idx]
            
            # Calcola gli assi ruotati
            x_axis = R_mean @ np.array([axis_length, 0, 0])
            y_axis = R_mean @ np.array([0, axis_length, 0])
            z_axis = R_mean @ np.array([0, 0, axis_length])
            
            # Plotta asse X (in tonalità più scura)
            ax.plot([origin[0], x_axis[0]], 
                    [origin[1], x_axis[1]], 
                    [origin[2], x_axis[2]], 
                    color=color, linewidth=3, linestyle='-', 
                    label=f'{node_name} - X', alpha=0.9)
            
            # Plotta asse Y (in tonalità media)
            ax.plot([origin[0], y_axis[0]], 
                    [origin[1], y_axis[1]], 
                    [origin[2], y_axis[2]], 
                    color=color, linewidth=3, linestyle='--', 
                    label=f'{node_name} - Y', alpha=0.7)
            
            # Plotta asse Z (in tonalità più chiara)
            ax.plot([origin[0], z_axis[0]], 
                    [origin[1], z_axis[1]], 
                    [origin[2], z_axis[2]], 
                    color=color, linewidth=3, linestyle=':', 
                    label=f'{node_name} - Z', alpha=0.6)
            
            # Aggiungi etichette agli estremi degli assi
            ax.text(x_axis[0], x_axis[1], x_axis[2], f'{node_name}-X', 
                    fontsize=9, color=color, weight='bold')
            ax.text(y_axis[0], y_axis[1], y_axis[2], f'{node_name}-Y', 
                    fontsize=8, color=color)
            ax.text(z_axis[0], z_axis[1], z_axis[2], f'{node_name}-Z', 
                    fontsize=8, color=color, style='italic')
        
        # Setup assi
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        ax.set_xlabel('X', fontsize=12, weight='bold')
        ax.set_ylabel('Y', fontsize=12, weight='bold')
        ax.set_zlabel('Z', fontsize=12, weight='bold')
        ax.set_title(f'Heading medio - {group_name.upper()} (primi 100 campioni)\nFrame 3D sovrapposti', 
                     fontsize=14, weight='bold')
        
        # Aggiungi griglia
        ax.grid(True, alpha=0.3)
        
        # Legenda
        ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
        
        # Imposta vista isometrica
        ax.view_init(elev=20, azim=45)
        
        plt.tight_layout()
        
        # Salva il grafico
        save_path = os.path.join(output_dir, f"mean_headings_3d_{group_name}.png")
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"     💾 Plot salvato: {save_path}")
    
    # Mostra tutti i plot
    print("\n   ▶️  Apertura finestre plot...")
    plt.show()

def main():
    """Main function."""
    # Directory con i file di input
    script_dir = os.path.dirname(os.path.abspath(__file__))
    inputs_dir = os.path.join(script_dir, "ExtractedNodeRawDataCSV")
    
    print("="*60)
    print("  FindImuWorld - Calcolo heading medio e visualizzazione 3D")
    print("="*60)
    
    # Carica i file dei nodi
    try:
        node_data = load_node_files(inputs_dir)
    except FileNotFoundError as e:
        print(f"\n❌ Errore: {e}")
        return
    
    # Calcola l'heading medio per ogni nodo
    mean_rotations, angles_data = compute_mean_heading_per_node(node_data, n_samples=100)
    
    if not mean_rotations:
        print("\n❌ Errore: nessun heading calcolato!")
        return
    
    # Stampa la procedura per tutti i nodi alla fine
    print("\n" + "="*80)
    print("  🔄 PROCEDURA ROTAZIONI INTRINSICHE (assi che si muovono)")
    print("="*80)
    
    for node_name in sorted(angles_data.keys()):
        roll, pitch, yaw = angles_data[node_name]['intrinsic']
        print(f"\n  📍 {node_name}:")
        print(f"     1️⃣  Parti dal frame di partenza (identità)")
        print(f"     2️⃣  Ruota di {roll:8.3f}° attorno all'asse X corrente")
        print(f"         → Gli assi Y e Z si muovono con il frame")
        print(f"     3️⃣  Ruota di {pitch:8.3f}° attorno al NUOVO asse Y (già ruotato)")
        print(f"         → Gli assi X e Z si muovono ancora")
        print(f"     4️⃣  Ruota di {yaw:8.3f}° attorno al NUOVO asse Z (dopo le prime 2 rotazioni)")
        print(f"         → Ottieni l'orientamento finale")
    
    # Plotta il grafico combinato (node10_4+, node3)
    plot_combined_filtered_nodes(mean_rotations, output_dir=script_dir)
    
    print("\n" + "="*60)
    print("  ✅ Elaborazione completata!")
    print("="*60)

if __name__ == "__main__":
    main()
