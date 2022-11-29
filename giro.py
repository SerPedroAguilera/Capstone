def giro(sentido, velocidad):
	#izquierda = 0
	#derecha = 1
	if sentido == 0:
		return [velocidad, 1.82*velocidad]
	elif sentido == 1:
		return [1.82*velocidad, velocidad]
