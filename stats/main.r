#Load data
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

#Mass
mass <- data$m..g.
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

n_d_M <- length(d_M)
error_3 <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_3 <- c(mean_d_M - error_3, mean_d_M + error_3)

boxplot(d_M, main="Boxplot de diámetros menores medidos en muestra", ylabel="Diámetros (cm)")
