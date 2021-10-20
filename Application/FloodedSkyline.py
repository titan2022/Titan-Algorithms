def floodedSkyline(heights):

    # GENERAL PROCEDURE
        # Get the total amount of floodable squares given the current heights arrangement
        # Iterate through the heights, creating a "hypothetical" for each in which the current height is set to 0
        # Get the total amount of floodable squares for this hypothetical
        # Append the difference between the hypothetical's total and the current version's total to the output array

    # GETTING THE TOTAL AMOUNT OF FLOODABLE SQUARES
        # Iterate through the given array
        # For each value, find the lesser of the maximum of the values to both its left and its right 
        #   This is the minimum height required to "level" the valley
        # Subtract the level value from the current height, giving the "depth" of the valley at the current index 
        #   This is how many squares it will take up in this column
        # Add all the non-negative depths together in order to get a total amount of squares that would be flooded, then return this value

    solution = []
    total = 0
    
    for i in range(len(heights)):
        lMax = 0
        rMax = 0
        for j in range(i):
            lMax = max(lMax, heights[j])
        for j in range(i + 1, len(heights)):
            rMax = max(rMax, heights[j])
        level = min(lMax, rMax)
        depth = level - heights[i]
        if depth > 0:
            total += depth

    for i in range(len(heights)):
        hypothetical = heights.copy()
        hypothetical[i] = 0
        hypoTotal = 0
        for i in range(len(hypothetical)):
            lMax = 0
            rMax = 0
            for j in range(i):
                lMax = max(lMax, hypothetical[j])
            for j in range(i + 1, len(hypothetical)):
                rMax = max(rMax, hypothetical[j])
            level = min(lMax, rMax)
            depth = level - hypothetical[i]
            if depth > 0:
                hypoTotal += depth
        solution.append(hypoTotal - total)

    return solution


def main():
    inp = [3, 1, 2, 1, 4, 3, 1, 2, 1]
    print(floodedSkyline(inp))

main()
