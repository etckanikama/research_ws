import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors

def load_csv_data(path, nrows=400):
    data = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i >= nrows:
                break
            data.append(row)
    return np.array(data, dtype=float)

if __name__ == "__main__":
    csv_path = "/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/20230826_boxel_nasi_0.0_0.0_0.0.csv" 
    data_np = load_csv_data(csv_path)
    values = data_np[:, 4]*400
    norm = colors.Normalize(vmin=values.min(), vmax=values.max())

    # 以下はカラーバーの表示部分です
    fig, ax = plt.subplots(figsize=(1, 5))
    fig.subplots_adjust(right=0.5)
    cmap = cm.get_cmap('viridis')
    cbar = plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, orientation='vertical')
    cbar.set_label('Value')
    plt.show()