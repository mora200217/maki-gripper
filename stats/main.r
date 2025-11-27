<<<<<<< HEAD
# Datos explÃ­citos
d_M <- c(24,19,19,17,18,23,24,25,21,29,
         25,20,21,29,17,14,15,11,25,24,
         19,19,21,19,20,22,25,22,19,19)

d_m <- c(20,19,18,17,18,20,22,23,22,23,
         25,19,21,19,16,14,14,10,22,23,
         19,19,18,18,19,20,21,21,18,19)
=======
# Maki Gripper 
# ---
# Tratamiento estadístico de muestras de tomate (n = 30) 

#Load data
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")
>>>>>>> 2d97c6fe14d9d5cb62a54a8179ea98680a22b40c

mass <- c(190,115,90,90,90,160,160,220,160,315,
          250,116,145,145,70,45,45,120,210,210,
          100,125,100,125,215,155,100,100,150,155)

<<<<<<< HEAD
# FunciÃ³n auxiliar para histograma con curva normal
plot_hist <- function(x, titulo, xlab) {
  mean_x <- mean(x)
  sd_x   <- sd(x)
  
  hist(x, breaks = 10, probability = TRUE,
       col = "lightblue", border = "black",
       main = titulo, xlab = xlab)
  
  curve(dnorm(x, mean = mean_x, sd = sd_x),
        col = "red", lwd = 2, add = TRUE)
  
  legend("topright",
         legend = c(paste0("Media = ", round(mean_x,2)),
                    paste0("Desv = ", round(sd_x,2))),
         bty = "n")
}

# Un solo grÃ¡fico con 3 subplots
par(mfrow = c(1,3))  # 1 fila, 3 columnas

plot_hist(mass, "DistribuciÃ³n de masas", "Masa (g)")
plot_hist(d_M,  "DistribuciÃ³n de diÃ¡metros mayores", "DiÃ¡metro mayor (cm)")
plot_hist(d_m,  "DistribuciÃ³n de diÃ¡metros menores", "DiÃ¡metro menor (cm)")

# Reset layout
par(mfrow = c(1,1))

=======
#Mass
mass <- data$m..g.
mean_mass <- mean(mass)
sd_mass <- sd(mass)
n_mass <- length(mass)
error_m <- qnorm(0.975) * sd_mass / sqrt(n_mass)
ci_mass <- c(mean_mass - error_m, mean_mass + error_m)

hist(mass, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de masas",
     xlab="Masa (g)")
x_mass <- seq(min(mass), max(mass), length=100)
lines(x_mass, dnorm(x_mass, mean=mean_mass, sd=sd_mass), col="red", lwd=2)
text(x = max(mass)*0.8, y = 0.012,
     labels = paste("Media =", round(mean_mass, 2),
                    "\nDesv =", round(sd_mass, 2)),
     pos = 4)

#Major diameter 
d_M <- (data$D_M..cm.)/(2*pi)
mean_d_M <- mean(d_M)
sd_d_M   <- sd(d_M)
n_d_M <- length(d_M)
error_d_M <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_d_M <- c(mean_d_M - error_d_M, mean_d_M + error_d_M)

hist(d_M, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros mayores",
     xlab="Diámetros (cm)")
x_d_M <- seq(min(d_M), max(d_M), length=100)
lines(x_d_M, dnorm(x_d_M, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)
text(x = max(d_M)*0.9, y = 0.7,
     labels = paste("Media =", round(mean_d_M, 2),
                    "\nDesv =", round(sd_d_M, 2)),
     pos = 4)

#Minor diameter 
d_m <- (data$d_m..cm.)/(2*pi)
mean_d_m <- mean(d_m)
sd_d_m  <- sd(d_m)
n_d_m <- length(d_m)
error_d_m <- qnorm(0.975) * sd_d_m / sqrt(n_d_m)
ci_d_m <- c(mean_d_m - error_d_m, mean_d_m + error_d_m)

hist(d_m, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros menores",
     xlab="Diámetros (cm)")
x_d_m <- seq(min(d_m), max(d_m), length=100)
lines(x_d_m, dnorm(x_d_m, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)
text(x = max(d_m)*0.8, y = 1.5,
     labels = paste("Media =", round(mean_d_m, 2),
                    "\nDesv =", round(sd_d_m, 2)),
     pos = 4)

# Inversa unilateral 
summary(data)

## Limpieza para masa

# Calcular Q1 y Q3
Q1_mass <- quantile(mass, 0.25)
Q3_mass <- quantile(mass, 0.75)
IQR_mass <- Q3_mass - Q1_mass

inf_mass <- Q1_mass - 1.5 * IQR_mass
sup_mass <- Q3_mass + 1.5 * IQR_mass

# Filtrar sin atípicos
mass 
mass_clean <- mass[mass >= inf_mass & mass <= sup_mass]
mass_clean

boxplot(main="Boxplot de masas medidas en muestra", ylab="Masa (g)", 
        list(Original = mass, Filtrado = mass_clean))

summary(mass_clean)
summary(mass)

## Limpieza para diámetro mayor

# Calcular Q1 y Q3
Q1_d_M <- quantile(d_M, 0.25)
Q3_d_M <- quantile(d_M, 0.75)
IQR_d_M <- Q3_d_M - Q1_d_M

inf_d_M <- Q1_d_M - 1.5 * IQR_d_M
sup_d_M <- Q3_d_M + 1.5 * IQR_d_M

# Filtrar sin atípicos
d_M
d_M_clean <- d_M[d_M >= inf_d_M & d_M <= sup_d_M]
d_M_clean

boxplot(main="Boxplot de diámetros mayores medidos en muestra", ylab="Diámetros (cm)", 
        list(Original = d_M, Filtrado = d_M_clean))

summary(d_M_clean)
summary(d_M)

## Limpieza para diámetro menor

# Calcular Q1 y Q3
Q1_d_m <- quantile(d_m, 0.25)
Q3_d_m <- quantile(d_m, 0.75)
IQR_d_m <- Q3_d_m - Q1_d_m

inf_d_m <- Q1_d_m - 1.5 * IQR_d_m
sup_d_m <- Q3_d_m + 1.5 * IQR_d_m

# Filtrar sin atípicos
d_m
d_m_clean <- d_m[d_m >= inf_d_m & d_m <= sup_d_m]
d_m_clean

boxplot(main="Boxplot de diámetros menores medidos en muestra", ylab="Diámetros (cm)", 
        list(Original = d_m, Filtrado = d_m_clean))

summary(d_m_clean)
summary(d_m)
>>>>>>> 2d97c6fe14d9d5cb62a54a8179ea98680a22b40c
