<<<<<<< HEAD
# Cargar datos
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

mass <- data$m..g.
=======
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
>>>>>>> 3cd7c2bde01346fcf8483e8960f5ec1f7316505e


mean_mass <- mean(mass)
sd_mass   <- sd(mass)


hist(mass, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de masas",
     xlab="Masa (g)")
x <- seq(min(mass), max(mass), length=100)
lines(x, dnorm(x, mean=mean_mass, sd=sd_mass), col="red", lwd=2)

n <- length(mass)
error <- qnorm(0.975) * sd_mass / sqrt(n)
ci <- c(mean_mass - error, mean_mass + error)
ci


boxplot(mass, main="Boxplot the masas medidas en muestra")

