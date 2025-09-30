# Prueba de R 
x <- c(5, 6, 5, 6, 7, 7, 5, 6, 7, 6, 7, 7, 8); 

mx = mean(x); 
sdx = sd(x); 

xdn <- seq(-5, 5, length.out = 100); 
y = dnorm(xdn, mx, sdx)

plot(xdn, y, main = "Distribucion normal para tomates"); 

print("Hola mundo")



