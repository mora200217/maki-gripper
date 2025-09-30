datos <- read.csv("G:/Universidad/Trabajos_Mecatronica/9_Semestre/PAI/Tomate.csv") 
#Mass
mass <- datos$`m..g.` 
m1 <- mean(x) 
sd1 <- sd(x)
hist(mass, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "Distribución de masas", xlab = "Masa (g)")  
curve(dnorm(x, mean = m1, sd = sd1), col = "red", lwd = 2, add = TRUE)
boxplot(mass,main = "Distribución de masas", ylab = "Masa (g)", xlab = "Muestras")
#Major diameter
d_M <- datos$`D_M..cm.` 
m2 <- mean(d_M) 
sd2 <- sd(d_M)
hist(d_M, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "Distribución de diámetros mayores", xlab = "Diámetros Mayores (cm)")  
curve(dnorm(x, mean = m2, sd = sd2), col = "red", lwd = 2, add = TRUE)
#Minor diameter
d_m <- datos$`d_m..cm.` 
m3 <- mean(d_m) 
sd3 <- sd(d_m)
hist(d_m, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "Distribución de diámetros menores", xlab = "Diámetros Menores (cm)")  
curve(dnorm(x, mean = m3, sd = sd3), col = "red", lwd = 2, add = TRUE)
boxplot(d_M, d_m,main = "Distribución de masas", ylab = "Masa (g)", xlab = "Muestras")



