// extern crate glutin_window;
// extern crate graphics;
// extern crate opengl_graphics;
// extern crate piston;

// use glutin_window::GlutinWindow as Window;
// use opengl_graphics::{GlGraphics, OpenGL};
// use piston::event_loop::{EventSettings, Events};
// use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
// use piston::window::WindowSettings;
// use std::{env, time::Instant};

// // Some color definitions for good measure
// // const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
// const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
// const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
// const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
// // const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
// const GOLD: [f32; 4] = [0.81176471, 0.72156863, 0.48627451, 1.0];
// const DARKGRAY: [f32; 4] = [0.3372549, 0.35294118, 0.36078431, 1.0];
// const LIGHTGRAY: [f32; 4] = [0.63529412, 0.64313725, 0.63921569, 1.0];

// pub struct WorldMapView {
//     gl: GlGraphics, // OpenGL drawing backend.
//     pose: Vec<f64>, // Rotation for the square.
//     width: f64,
//     timer: Instant,
// }

// impl WorldMapView {
//     fn render(&mut self, args: &RenderArgs) {
//         use graphics::*;

//         let square1 = rectangle::square(self.pose[0], self.pose[1], self.width);
//         let square2 = rectangle::square(self.pose[0], self.pose[1], self.width * 1.15);
//         let rotation = self.pose[2];
//         let (x, y) = (args.window_size[0] / 2.0, args.window_size[1] / 2.0);

//         self.gl.draw(args.viewport(), |c, gl| {
//             // Clear the screen.
//             clear(DARKGRAY, gl);

//             let transform1 = c
//                 .transform
//                 .trans(x, y)
//                 .rot_rad(rotation)
//                 .trans(-self.width / 2.0, -self.width / 2.0);

//             let transform2 = c
//                 .transform
//                 .trans(x, y)
//                 .rot_rad(rotation)
//                 .trans(-self.width * 1.15 / 2.0, -self.width * 1.15 / 2.0);

//             // Draw a box rotating around the middle of the screen.
//             rectangle(GOLD, square2, transform2, gl);
//             rectangle(BLACK, square1, transform1, gl);
//         });
//     }

//     fn update(&mut self, args: &UpdateArgs) {
//         // Rotate 2 radians per second.
//         let t = self.timer.elapsed().as_millis() as f64;
//         self.pose[0] += (t / 1000.0).sin() / 10.0;
//         self.pose[1] += (t / 1000.0).cos() / 10.0;
//         self.pose[2] += 2.0 * args.dt;
//     }
// }

// use plotters::prelude::*;

// fn snowflake_iter(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
//     let mut ret = vec![];
//     for i in 0..points.len() {
//         let (start, end) = (points[i], points[(i + 1) % points.len()]);
//         let t = ((end.0 - start.0) / 3.0, (end.1 - start.1) / 3.0);
//         let s = (
//             t.0 * 0.5 - t.1 * (0.75f64).sqrt(),
//             t.1 * 0.5 + (0.75f64).sqrt() * t.0,
//         );
//         ret.push(start);
//         ret.push((start.0 + t.0, start.1 + t.1));
//         ret.push((start.0 + t.0 + s.0, start.1 + t.1 + s.1));
//         ret.push((start.0 + t.0 * 2.0, start.1 + t.1 * 2.0));
//     }
//     ret
// }

// const OUT_FILE_NAME: &'static str =
//     &format!("{}/data/control.gif", env::var("PROJECT_ROOT").unwrap()).as_str();
// fn sample_plot() -> Result<(), Box<dyn std::error::Error>> {
//     let root = BitMapBackend::gif(OUT_FILE_NAME, (800, 600), 1_000)?.into_drawing_area();

//     for i in 0..8 {
//         root.fill(&WHITE)?;

//         let mut chart = ChartBuilder::on(&root)
//             .caption(
//                 format!("Koch's Snowflake (n_iter = {})", i),
//                 ("sans-serif", 50),
//             )
//             .build_cartesian_2d::<f32, f32>(-2.0..2.0, -1.5..1.5)?;

//         let mut snowflake_vertices = {
//             let mut current: Vec<(f64, f64)> = vec![
//                 (0.0, 1.0),
//                 ((3.0f64).sqrt() / 2.0, -0.5),
//                 (-(3.0f64).sqrt() / 2.0, -0.5),
//             ];
//             for _ in 0..i {
//                 current = snowflake_iter(&current[..]);
//             }
//             current
//         };

//         chart.draw_series(std::iter::once(Polygon::new(
//             snowflake_vertices.clone(),
//             &RED,
//         )))?;

//         snowflake_vertices.push(snowflake_vertices[0]);
//         chart.draw_series(std::iter::once(PathElement::new(snowflake_vertices, &RED)))?;

//         root.present()?;
//     }

//     println!("Result has been saved to {}", OUT_FILE_NAME);

//     Ok(())
// }
