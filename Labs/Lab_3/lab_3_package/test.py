from costmap_no_postbox import data
import matplotlib.pyplot as plt

def main():
    print("TESTING")
    print(data)
    print(len(data))


 plt.scatter([i[0] for i in simulated_points], [i[1] for i in simulated_points], color="red")
            # plt.xlim(-5, 6)
            # plt.ylim(-10, 15)
            # plt.xlim(-1000, 1000)
            # plt.ylim(-1000, 1000)
            plt.show()


if __name__ == '__main__':
    main()
