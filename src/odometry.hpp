		// double xval = pros::c::adi_encoder_get(xenc);
		// double yval = -pros::c::adi_encoder_get(yenc);
		// double deltaTheta = theta - prevtheta;

		// double localOffX = (xval - prevx) / 360.0 * 1_pi * (3.5_in).convert(okapi::meter);
		// double localOffY = (yval - prevy) / 360.0 * 1_pi * (3.5_in).convert(okapi::meter);

		// double avgA = (((deltaTheta / 2) + prevtheta) * okapi::degree).convert(okapi::radian);

		// double polarR = std::sqrt(localOffX * localOffX + localOffY * localOffY);
		// double polarA = std::atan2(localOffY, localOffX) - avgA;

		// double dX = std::sin(polarA) * polarR;
		// double dY = std::cos(polarA) * polarR;

		// if (isnan(dX))
		//{
		//	dX = 0;
		// }

		// if (isnan(dY))
		//{
		//	dY = 0;
		// }

		// if (isnan(deltaTheta))
		//{
		//	deltaTheta = 0;
		// }

		// state.x += dX * okapi::meter, state.y += dY * okapi::meter, state.theta = theta * okapi::degree;
		// if (!(count_loops % 70))
		//{
		//	printf("%f %f\n", std::atan2(localOffY, localOffX), avgA);
		//	printf("%f %f tiles %f degree\n", state.x / 24_in, state.y / 24_in, state.theta.convert(okapi::degree));
		// }

		//// odom stuff

		// prevtheta = theta;
		// prevx = xval;
		// prevy = yval;

