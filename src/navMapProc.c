#define MAXSIZE 1024
typedef unsigned char uint8_t;

#define CHK_OR_NEIGHBOURS(arr, i, j, val) \
        ((arr)[(i)-1][(j)-1]==(val) || (arr)[(i)-1][(j)]==(val) || (arr)[(i)-1][(j)+1]==(val) || arr[(i)][(j)-1]==(val) ||\
        (arr)[(i)][(j)+1]==(val) || (arr)[(i)+1][(j)-1]==(val) || (arr)[(i)+1][(j)]==(val) || arr[(i)+1][(j)+1]==(val))

void MapDilateErode (uint8_t *mapArr, int height, int width)
{
    uint8_t tmpMapArr[MAXSIZE][MAXSIZE]; 
    for (int i = 1; i < height - 1; i++)
        for (int j = 1; j < width - 1; j++)
            tmpMapArr[i][j] = mapArr[i*width+j];
    for (int i = 1; i < height - 1; i++) {
        for (int j = 1; j < width - 1; j++) {
            if (tmpMapArr[i][j] != 2 && CHK_OR_NEIGHBOURS(tmpMapArr, i, j, 2))
                mapArr[i*width+j] = 2;  // Dilate the walls
            else if (tmpMapArr[i][j] == 0 && CHK_OR_NEIGHBOURS(tmpMapArr, i, j, 1))
                mapArr[i*width+j] = 1;  // Erode the unmapped
        }
    }
}