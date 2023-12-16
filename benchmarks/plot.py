import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot(data):
    fig = plt.figure(figsize=(16, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')

    x = data['minCostFlowDijkstra'][1]
    y = data['minCostFlowDijkstra'][2]

    dijkstra = data['minCostFlowDijkstra'][0]
    edmonds_karp = data['minCostFlowEdmondsKarp'][0]
    bottom = np.zeros_like(dijkstra)
    width = 150
    depth = 3

    ax1.bar3d(x, y, bottom, width, depth, dijkstra, shade=True)
    ax1.set_title('Dijkstra')

    ax2.bar3d(x, y, bottom, width, depth, edmonds_karp, shade=True)
    ax2.set_title('Edmonds-Karp')

    for ax in [ax1, ax2]:
        ax.set_xlabel("Vertices")
        ax.set_ylabel("Percentage")
        ax.set_zlabel("CPU time")

    plt.show()


def parse_csv(csv_file):
    data = dict()
    for name, cpu_time in pd.read_csv(csv_file,
                                      delimiter=',',
                                      skiprows=11,  # FIXME: well done out
                                      skipinitialspace=True,
                                      usecols=['name', 'cpu_time']).values:
        splitted = name.split('/')
        if splitted[0] not in data:
            data[splitted[0]] = ([], [], [])
        data[splitted[0]][0].append(cpu_time)
        data[splitted[0]][1].append(int(splitted[1]))
        data[splitted[0]][2].append(int(splitted[2]))
    return data


def main():
    parser = argparse.ArgumentParser(
        prog='BenchmarkPlot',
        description='Plot stats from csv GoogleBenchmark output')
    parser.add_argument(
        '-f',
        metavar="FILE",
        type=argparse.FileType("r"),
        default=sys.stdin,
        dest="file",
        help='Path to .cvs stats file')
    args = parser.parse_args()
    plot(parse_csv(args.file))


if __name__ == '__main__':
    main()
