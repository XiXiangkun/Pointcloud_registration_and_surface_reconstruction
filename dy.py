import tkinter as tk
import os

window=tk.Tk()
window.title("点云输入")
window.geometry("500x220")

spa = tk.Label(window, text=' ', font=('Arial', 12), width=30, height=2)
spa.pack()
tk.Label(window,text='点云1路径：',font=('Arial', 12)).place(x=40,y=40)
tk.Label(window,text='点云2路径：',font=('Arial', 12)).place(x=40,y=80)


var_get1=tk.StringVar()
var_get2=tk.StringVar()
e1=tk.Entry(window,show=None,textvariable=var_get1,font=('Arial', 12))
e2=tk.Entry(window,show=None,textvariable=var_get2,font=('Arial', 12))
e1.place(x=170,y=40)
e2.place(x=170,y=80)

def peizhun():
    get1=var_get1.get()
    get2=var_get2.get()
    var=tk.StringVar()

    def p_s():
        g=var.get()
        print(g)
        os.system("ConsoleApplication1.exe -"+str(g)+" "+str(get1)+" "+str(get2))
        window_p.destroy()

    window_p = tk.Toplevel(window)
    window_p.geometry("500x200")
    window_p.title("配准算法选择")
    r1 = tk.Radiobutton(window_p, text="-p1（ICP）", variable=var, value="p1")
    r1.place(x=40,y=10)
    r2 = tk.Radiobutton(window_p, text="-p2（LM改进ICP）", variable=var, value="p2")
    r2.place(x=40, y=50)
    r3 = tk.Radiobutton(window_p, text="-sp1（下采样+PFH+SAC-IA+ICP）", variable=var, value="sp1")
    r3.place(x=40, y=100)
    r4 = tk.Radiobutton(window_p, text="-sp2（SIFT+PFH+SAC-IA+ICP）", variable=var, value="sp2")
    r4.place(x=40, y=150)
    b1=tk.Button(window_p,text="配准",font=('Arial', 12), width=8, height=1, command=p_s)
    b1.place(x=350,y=140)

def chongjian():
    get1 = var_get1.get()
    var = tk.StringVar()

    def c_j():
        g=var.get()
        print(g)
        os.system("ConsoleApplication1.exe -" + str(g) + " " + str(get1))
        window_c.destroy()

    window_c = tk.Toplevel(window)
    window_c.geometry("500x200")
    window_c.title("重建算法选择")
    r1 = tk.Radiobutton(window_c, text="-q1（MC）", variable=var, value="q1")
    r1.place(x=40, y=10)
    r2 = tk.Radiobutton(window_c, text="-q2（Poisson）", variable=var, value="q2")
    r2.place(x=40, y=50)
    r3 = tk.Radiobutton(window_c, text="-q3（贪婪投影三角化）", variable=var, value="q3")
    r3.place(x=40, y=100)
    b1 = tk.Button(window_c, text="重建", font=('Arial', 12), width=8, height=1, command=c_j)
    b1.place(x=350, y=140)



b1=tk.Button(window,text="配准",font=('Arial', 12), width=10, height=1, command=peizhun)
b1.place(x=120,y=130)
b2=tk.Button(window,text="重建",font=('Arial', 12), width=10, height=1, command=chongjian)
b2.place(x=280,y=130)

 
# 第6步，主窗口循环显示
window.mainloop()
