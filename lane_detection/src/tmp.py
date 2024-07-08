        # -------------  ------------------ # 

        histogram = np.sum(img[img.shape[0] // 2:, :], axis=0)

        midpoint = 320
        leftx_current = np.argmax(histogram[:250])
        rightx_current = np.argmax(histogram[midpoint+80:]) + midpoint+80

        if (abs(leftx_current - x_temp) >30):
            leftx_current = x_temp
        if (abs(rightx_current-y_temp)>30):
            rightx_current= y_temp

        y_temp = rightx_current    
        x_temp = leftx_current 
        nz = img.nonzero()

        # --------------------------------- # 