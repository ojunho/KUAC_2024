

class Object:
    def __init__(self, x):
        self.x = x
sorted_ar_list = []

# for i in range(5):
#     new_obj = Object(i)
#     sorted_ar_list.append(new_obj)

sorted_ar_list.sort(key=lambda x: x.x)


print(sorted_ar_list)