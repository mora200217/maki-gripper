# Cargar datos
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

mass <- data$m..g.


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

