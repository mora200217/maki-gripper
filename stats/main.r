# Maki Gripper 
# ---
# Tratamiento estad�stico de muestras de tomate (n = 30) 

#Load data
data <- read.csv("dataset-28-09-25.csv", header = TRUE, sep = ",")

head(data)

#Mass
mass <- data$m..g.
mean_mass <- mean(mass)
sd_mass <- sd(mass)

hist(mass, breaks=10, probability=TRUE,
     col="lightblue", main="Distribuci�n de masas",
     xlab="Masa (g)")
x_mass <- seq(min(mass), max(mass), length=100)
lines(x_mass, dnorm(x_mass, mean=mean_mass, sd=sd_mass), col="red", lwd=2)

n_mass <- length(mass)
error_m <- qnorm(0.975) * sd_mass / sqrt(n_mass)
ci_mass <- c(mean_mass - error_m, mean_mass + error_m)


#Major diameter 
d_M <- (data$D_M..cm.)/(2*pi)
mean_d_M <- mean(d_M)
sd_d_M   <- sd(d_M)

hist(d_M, breaks=10, probability=TRUE,
     col="lightblue", main="Distribuci�n de di�metros mayores",
     xlab="Di�metros (cm)")
x_d_M <- seq(min(d_M), max(d_M), length=100)
lines(x_d_M, dnorm(x_d_M, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_M <- length(d_M)
error_d_M <- qnorm(0.975) * sd_d_M / sqrt(n_d_M)
ci_d_M <- c(mean_d_M - error_d_M, mean_d_M + error_d_M)

#Minor diameter 
d_m <- (data$d_m..cm.)/(2*pi)
mean_d_m <- mean(d_m)
sd_d_m  <- sd(d_m)

hist(d_m, breaks=10, probability=TRUE,
     col="lightblue", main="Distribuci�n de di�metros menores",
     xlab="Di�metros (cm)")
x_d_m <- seq(min(d_m), max(d_m), length=100)
lines(x_d_m, dnorm(x_d_m, mean=mean_d_M, sd=sd_d_M), col="red", lwd=2)

n_d_m <- length(d_m)
error_d_m <- qnorm(0.975) * sd_d_m / sqrt(n_d_m)
ci_d_m <- c(mean_d_m - error_d_m, mean_d_m + error_d_m)

# Inversa unilateral 
summary(data)

## Limpieza para masa

# Calcular Q1 y Q3
mass 
Q1_mass <- quantile(mass, 0.25)
Q3_mass <- quantile(mass, 0.75)
IQR_mass <- Q3_mass - Q1_mass

inf_mass <- Q1_mass - 1.5 * IQR_mass
sup_mass <- Q3_mass + 1.5 * IQR_mass

# Filtrar sin at�picos
mass 
mass_clean <- mass[mass >= inf_mass & mass <= sup_mass]
mass_clean

boxplot(main="Boxplot de masas medidas en muestra", ylab="Masa (g)", 
        list(Original = mass, Limpio = mass_clean))

summary(mass_clean)
summary(mass)

## Limpieza para di�metro mayor

# Calcular Q1 y Q3
d_M
Q1_d_M <- quantile(d_M, 0.25)
Q3_d_M <- quantile(d_M, 0.75)
IQR_d_M <- Q3_d_M - Q1_d_M

inf_d_M <- Q1_d_M - 1.5 * IQR_d_M
sup_d_M <- Q3_d_M + 1.5 * IQR_d_M

# Filtrar sin at�picos
d_M
d_M_clean <- d_M[d_M >= inf_d_M & d_M <= sup_d_M]
d_M_clean

boxplot(main="Boxplot de di�metros mayores medidos en muestra", ylab="Di�metros (cm)", 
        list(Original = d_M, Limpio = d_M_clean))

summary(d_M_clean)
summary(d_M)

## Limpieza para di�metro menor

# Calcular Q1 y Q3
d_m
Q1_d_m <- quantile(d_m, 0.25)
Q3_d_m <- quantile(d_m, 0.75)
IQR_d_m <- Q3_d_m - Q1_d_m

inf_d_m <- Q1_d_m - 1.5 * IQR_d_m
sup_d_m <- Q3_d_m + 1.5 * IQR_d_m

# Filtrar sin at�picos
d_m
d_m_clean <- d_m[d_m >= inf_d_m & d_m <= sup_d_m]
d_m_clean

boxplot(main="Boxplot de di�metros menores medidos en muestra", ylab="Di�metros (cm)", 
        list(Original = d_m, Limpio = d_m_clean))

summary(d_m_clean)
summary(d_m)
