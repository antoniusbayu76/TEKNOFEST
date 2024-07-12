from PIL import Image, ImageFilter, ImageEnhance
import numpy as np

def apply_underwater_filter(image_path, output_path):
    # Open an image file
    with Image.open(image_path) as img:
        # Convert the image to RGB if it's not already
        img = img.convert("RGB")

        # Apply a blue-green color tint
        np_img = np.array(img)
        blue_green_tint = np.array([0.0, 1.0, 1.5])
        np_img = np.clip(np_img * blue_green_tint, 0, 255).astype(np.uint8)

        img = Image.fromarray(np_img)

        # Apply a slight blur to mimic the water effect
        img = img.filter(ImageFilter.GaussianBlur(radius=1))  # Increase radius for smoother effect

        # Slightly enhance the brightness to mimic light refraction
        enhancer = ImageEnhance.Brightness(img)
        img = enhancer.enhance(1.1)  # Slightly decrease to avoid over-brightening

        # Optionally apply a small ripple effect (requires scipy)
        try:
            from scipy.ndimage import map_coordinates

            def ripple_effect(image, alpha=2, wavelength=30):
                """ Apply a ripple effect to an image. """
                x, y, z = image.shape
                xx, yy = np.meshgrid(np.arange(x), np.arange(y), indexing='ij')
                xx = xx + alpha * np.sin(2 * np.pi * yy / wavelength)
                yy = yy + alpha * np.sin(2 * np.pi * xx / wavelength)

                coords = np.array([xx, yy])
                distorted_img = np.empty_like(image)
                for i in range(z):
                    distorted_img[..., i] = map_coordinates(image[..., i], [coords[0], coords[1]], order=1, mode='reflect')
                return distorted_img

            np_img = np.array(img)
            np_img = ripple_effect(np_img)
            img = Image.fromarray(np_img)
        except ImportError:
            print("Scipy is not installed. Ripple effect will be skipped.")

        # Save the modified image
        img.save(output_path)

# Example usage
apply_underwater_filter("target.jpg", "targetair.jpg")
