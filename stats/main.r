<<<<<<< HEAD
# Maki Gripper 
# ---
# Tratamiento estadÃ­stico de muestras de tomate (n = 30) 

=======
#Load data
>>>>>>> 8ef1ac1f20818251af40a80114ce19617063e8ae
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

<<<<<<< HEAD
# === MASAS 

mass <- datos$`m..g.` 
m1 <- mean(x) 
sd1 <- sd(x)
hist(mass, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "DistribuciÃ³n de masas", xlab = "Masa (g)")  
curve(dnorm(x, mean = m1, sd = sd1), col = "red", lwd = 2, add = TRUE)
boxplot(mass,main = "DistribuciÃ³n de masas", ylab = "Masa (g)", xlab = "Muestras")


d_M <- datos$`D_M..cm.` 
m2 <- mean(d_M) 
sd2 <- sd(d_M)
hist(d_M, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "DistribuciÃ³n de diÃ¡metros mayores", xlab = "DiÃ¡metros Mayores (cm)")  
curve(dnorm(x, mean = m2, sd = sd2), col = "red", lwd = 2, add = TRUE)

d_m <- datos$`d_m..cm.` 
m3 <- mean(d_m) 
sd3 <- sd(d_m)
hist(d_m, breaks = 10, probability = TRUE, col = "lightblue", border = "black", main = "DistribuciÃ³n de diÃ¡metros menores", xlab = "DiÃ¡metros Menores (cm)")  
curve(dnorm(x, mean = m3, sd = sd3), col = "red", lwd = 2, add = TRUE)
boxplot(d_M, d_m,main = "DistribuciÃ³n de masas", ylab = "Masa (g)", xlab = "Muestras")

=======
#Mass
mass <- data$m..g.
>>>>>>> 8ef1ac1f20818251af40a80114ce19617063e8ae
mean_mass <- mean(mass)
sd_mass   <- sd(mass)

hist(mass, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de masas",
     xlab="Masa (g)")
x1 <- seq(min(mass), max(mass), length=100)
lines(x1, dnorm(x1, mean=mean_mass, sd=sd_mass), col="red", lwd=2)

n_mass <- length(mass)
error <- qnorm(0.975) * sd_mass / sqrt(n_mass)
ci <- c(mean_mass - error, mean_mass + error)

boxplot(mass, main="Boxplot de masas medidas en muestra", ylabel="Masa (g)")

#Major diameter 
d_M <- data$D_M..cm.
mean_d_M <- mean(d_M)
sd_d_M   <- sd(d_M)

hist(d_M, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros mayores",
     xlab="Diámetros (cm)")
x2 <- seq(min(d_M), max(d_M), length=100)
lines(x2, dnorm(x2, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_M <- length(d_M)
error_2 <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_2 <- c(mean_d_M - error_2, mean_d_M + error_2)

boxplot(d_M, main="Boxplot de diámetros mayores medidos en muestra", ylabel="Diámetros (cm)")

#Minor diameter 
d_m <- data$d_m..cm.
mean_d_m <- mean(d_m)
sd_d_m  <- sd(d_m)


hist(d_m, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros menores",
     xlab="Diámetros (cm)")
x <- seq(min(d_m), max(d_m), length=100)
lines(x, dnorm(x, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

<<<<<<< HEAD

# Inversa unilateral 
summary(data)

## Limpieza para masa

# Calcular Q1 y Q3
mass 
Q1 <- quantile(mass, 0.25)
Q3 <- quantile(mass, 0.75)
IQR <- Q3 - Q1


lim_inf <- Q1 - 1.5 * IQR
lim_sup <- Q3 + 1.5 * IQR

# Filtrar sin atÃ­picos
mass 
mass_clean <- mass[mass >= lim_inf & mass <= lim_sup]
mass_clean

boxplot(list(Original = mass, Limpio = mass_clean))

summary(mass_clean)
summary(mass)

=======
n_d_M <- length(d_M)
error_3 <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_3 <- c(mean_d_M - error_3, mean_d_M + error_3)

boxplot(d_M, main="Boxplot de diámetros menores medidos en muestra", ylabel="Diámetros (cm)")
>>>>>>> 8ef1ac1f20818251af40a80114ce19617063e8ae
