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
lines(x1, dnorm(x, mean=mean_mass, sd=sd_mass), col="red", lwd=2)

n_mass <- length(mass)
error <- qnorm(0.975) * sd_mass / sqrt(n)
ci <- c(mean_mass - error, mean_mass + error)

boxplot(mass, main="Boxplot the masas medidas en muestra", ylabel="Gramos(g)")
#Major diameter 

d_M <- data$D_M..cm.
mean_d_M <- mean(d_M)
sd_d_M   <- sd(d_M)


hist(d_M, breaks=10, probability=TRUE,
     col="lightblue", main="Distribución de diámetros mayores",
     xlab="Masa (g)")
x <- seq(min(d_M), max(d_M), length=100)
lines(x, dnorm(x, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_M <- length(d_M)
error_2 <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_2 <- c(mean_d_M - error_2, mean_d_M + error_2)

boxplot(d_M, main="Boxplot the masas medidas en muestra", ylabel="Gramos(g)")
