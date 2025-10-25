#Load data
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

#Mass
mass <- data$m..g.
mean_mass <- mean(mass)
sd_mass <- sd(mass)

hist(mass, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de masas",
     xlab="Masa (g)")
x_mass <- seq(min(mass), max(mass), length=100)
lines(x_mass, dnorm(x_mass, mean=mean_mass, sd=sd_mass), col="red", lwd=2)

n_mass <- length(mass)
error_m <- qnorm(0.975) * sd_mass / sqrt(n_mass)
ci_mass <- c(mean_mass - error_m, mean_mass + error_m)

boxplot(mass, main="Boxplot de masas medidas en muestra", ylabel="Masa (g)")

#Major diameter 
d_M <- (data$D_M..cm.)/(2*pi)
mean_d_M <- mean(d_M)
sd_d_M   <- sd(d_M)

hist(d_M, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros mayores",
     xlab="Diámetros (cm)")
x_d_M <- seq(min(d_M), max(d_M), length=100)
lines(x_d_M, dnorm(x_d_M, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_M <- length(d_M)
error_d_M <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_d_M <- c(mean_d_M - error_d_M, mean_d_M + error_d_M)

boxplot(d_M, main="Boxplot de diámetros mayores medidos en muestra", ylabel="Diámetros (cm)")

#Minor diameter 
d_m <- (data$d_m..cm.)/(2*pi)
mean_d_m <- mean(d_m)
sd_d_m  <- sd(d_m)

hist(d_m, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros menores",
     xlab="Diámetros (cm)")
x_d_m <- seq(min(d_m), max(d_m), length=100)
lines(x_d_m, dnorm(x_d_m, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_m <- length(d_m)
error_d_m <- qnorm(0.975) * sd_d_m / sqrt(n_d_m)
ci_d_m <- c(mean_d_m - error_d_m, mean_d_m + error_d_m)

boxplot(d_m, main="Boxplot de diámetros menores medidos en muestra", ylabel="Diámetros (cm)")
