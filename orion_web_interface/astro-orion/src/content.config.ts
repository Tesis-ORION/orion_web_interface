import { defineCollection, z } from "astro:content";

const demo = defineCollection({
  type: "data",
  schema: ({ image }) =>
    z.object({
      title: z.string().min(1),
      image: image(),
      url: z.string().url(),
      featured: z.number().min(1).optional(),
    }),
});

const about = defineCollection({
  schema: z.object({
    name: z.string(),
    careers: z.array(z.string()),
    description: z.string(),
    img: z.string().url(),
    github: z.string(),
  }),
});


export const collections = {
  demo,
  about,
};
