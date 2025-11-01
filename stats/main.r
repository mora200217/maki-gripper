# Datos explícitos
d_M <- c(24,19,19,17,18,23,24,25,21,29,
         25,20,21,29,17,14,15,11,25,24,
         19,19,21,19,20,22,25,22,19,19)

d_m <- c(20,19,18,17,18,20,22,23,22,23,
         25,19,21,19,16,14,14,10,22,23,
         19,19,18,18,19,20,21,21,18,19)

mass <- c(190,115,90,90,90,160,160,220,160,315,
          250,116,145,145,70,45,45,120,210,210,
          100,125,100,125,215,155,100,100,150,155)

# Función auxiliar para histograma con curva normal
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

# Un solo gráfico con 3 subplots
par(mfrow = c(1,3))  # 1 fila, 3 columnas

plot_hist(mass, "Distribución de masas", "Masa (g)")
plot_hist(d_M,  "Distribución de diámetros mayores", "Diámetro mayor (cm)")
plot_hist(d_m,  "Distribución de diámetros menores", "Diámetro menor (cm)")

# Reset layout
par(mfrow = c(1,1))

