import cv2
import os
def listdir(path, list_name):
    for file in os.listdir(path):
        file_path = os.path.join(path, file)
        if os.path.isdir(file_path):
            listdir(file_path, list_name)
        elif os.path.splitext(file_path)[1]=='.jpeg':
            list_name.append(file_path)
    return list_name

#root, files = file_name("D:/毕设/test/Food_waste_fruit")[:2]
list = []
list_file = []
num = 0
list_file = os.listdir('./')
for i in range(2,6):
    list = listdir('./' + list_file[i],list)#获取该目录下所有文件路径，存入列表中
    print(len(list))
    path = list_file[i]
    fileList = os.listdir(path)
    n = 0
    for i in fileList:
        #设置旧文件名（就是路径+文件名）
        oldname = path + os.sep + fileList[n]   # os.sep添加系统分隔符

        #设置新文件名
        newname = path + os.sep + str(n+1)+'.jpeg'

        os.rename(oldname, newname)   #用os模块中的rename方法对文件改名
        print(oldname, '======>', newname)

        n+=1

    for j in range(len(list)):
        img = cv2.imread(list[j])
        img1 = cv2.resize(img, (224, 224))
        cv2.imwrite(list[j], img1)
        num += 1
        print(num)
    list = []



#img = cv2.imread(root+files)


